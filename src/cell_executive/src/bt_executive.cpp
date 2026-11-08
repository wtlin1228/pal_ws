// Ch 17 (optional): the Ch 13 loop as a BehaviorTree.CPP tree. Sketch of the structure; needs behaviortree_cpp
// and an Executive whose helpers (clearFrame, capture, waitFrame, planPick, runPick, dets) are public.
#include <chrono>
#include <future>
#include <thread>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include "executive.hpp"               // the Executive class of Ch 13, moved to a header

using namespace BT;

class Capture : public SyncActionNode {
public:
  Capture(const std::string & name, const NodeConfig & cfg, Executive & ex) : SyncActionNode(name, cfg), ex_(ex) {}
  static PortsList providedPorts() { return {}; }
  NodeStatus tick() override { ex_.clearFrame(); return ex_.capture() ? NodeStatus::SUCCESS : NodeStatus::FAILURE; }
private:
  Executive & ex_;
};

class Detect : public StatefulActionNode {              // RUNNING until a frame arrives, FAILURE after 2 s
public:
  Detect(const std::string & name, const NodeConfig & cfg, Executive & ex) : StatefulActionNode(name, cfg), ex_(ex) {}
  static PortsList providedPorts() { return {}; }
  NodeStatus onStart() override { t0_ = std::chrono::steady_clock::now(); return NodeStatus::RUNNING; }
  NodeStatus onRunning() override {
    if (const auto f = ex_.waitFrame(std::chrono::milliseconds(0))) { ex_.dets = *f; return NodeStatus::SUCCESS; }
    return std::chrono::steady_clock::now() - t0_ < std::chrono::seconds(2) ? NodeStatus::RUNNING : NodeStatus::FAILURE;
  }
  void onHalted() override {}
private:
  Executive & ex_;
  std::chrono::steady_clock::time_point t0_;
};

class PalletEmpty : public ConditionNode {
public:
  PalletEmpty(const std::string & name, const NodeConfig & cfg, Executive & ex) : ConditionNode(name, cfg), ex_(ex) {}
  static PortsList providedPorts() { return {}; }
  NodeStatus tick() override { return ex_.dets.detections.empty() ? NodeStatus::SUCCESS : NodeStatus::FAILURE; }
private:
  Executive & ex_;
};

class PickPlaceLeaf : public StatefulActionNode {       // sends the goal on start, polls the result while running
public:
  PickPlaceLeaf(const std::string & name, const NodeConfig & cfg, Executive & ex) : StatefulActionNode(name, cfg), ex_(ex) {}
  static PortsList providedPorts() { return {}; }
  NodeStatus onStart() override {
    const auto goal = ex_.planPick(ex_.dets);
    if (!goal) return NodeStatus::FAILURE;
    result_ = std::async(std::launch::async, [this, g = *goal]() { return ex_.runPick(g).first; });
    return NodeStatus::RUNNING;
  }
  NodeStatus onRunning() override {
    if (result_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return NodeStatus::RUNNING;
    return result_.get() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }
  void onHalted() override {}
private:
  Executive & ex_;
  std::future<bool> result_;
};

static const char * TREE = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="OneBox">
    <Sequence>
      <Capture/>
      <Detect/>
      <Fallback>
        <PalletEmpty/>
        <RetryUntilSuccessful num_attempts="3">
          <PickPlace/>
        </RetryUntilSuccessful>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>)";

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto ex = std::make_shared<Executive>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(ex);
  std::thread spinner([&executor]() { executor.spin(); });

  BehaviorTreeFactory factory;
  factory.registerBuilder<Capture>("Capture", [&](const std::string & n, const NodeConfig & c) { return std::make_unique<Capture>(n, c, *ex); });
  factory.registerBuilder<Detect>("Detect", [&](const std::string & n, const NodeConfig & c) { return std::make_unique<Detect>(n, c, *ex); });
  factory.registerBuilder<PalletEmpty>("PalletEmpty", [&](const std::string & n, const NodeConfig & c) { return std::make_unique<PalletEmpty>(n, c, *ex); });
  factory.registerBuilder<PickPlaceLeaf>("PickPlace", [&](const std::string & n, const NodeConfig & c) { return std::make_unique<PickPlaceLeaf>(n, c, *ex); });
  auto tree = factory.createTreeFromText(TREE);
  printTreeRecursively(tree.rootNode());

  int moved = 0;
  while (rclcpp::ok()) {                                 // tick until the pallet is empty
    const NodeStatus status = tree.tickWhileRunning();
    if (status == NodeStatus::SUCCESS) { if (ex->dets.detections.empty()) break; ++moved; }
    else { RCLCPP_ERROR(ex->get_logger(), "tree failed: a person needs to look"); break; }
  }
  RCLCPP_INFO(ex->get_logger(), "done after %d boxes", moved);
  rclcpp::shutdown();
  spinner.join();
}
