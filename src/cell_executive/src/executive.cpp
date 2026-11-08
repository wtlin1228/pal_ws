// Ch 13-18: depalletizing and palletizing jobs (single and mixed SKU), with recovery and overlapped capture as an explicit state machine behind a RunJob action.
// Beliefs only: the last detection frame, the SKU table, and what it placed where. It never reads world_model.
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cell_interfaces/action/pick_place.hpp>
#include <cell_interfaces/action/run_job.hpp>
#include <cell_planning/geometry.hpp>
#include <cell_planning/grasps.hpp>
#include <cell_planning/skus.hpp>
#include <cell_planning/palletizer.hpp>
#include <cell_planning/mixed.hpp>
#include <cell_executive/scene.hpp>
#include <cell_motion/stamped.hpp>

using namespace std::chrono_literals;
using namespace cell_planning;
using cell_motion::stamped;
using PickPlace = cell_interfaces::action::PickPlace;
using RunJob = cell_interfaces::action::RunJob;
using JobHandle = rclcpp_action::ServerGoalHandle<RunJob>;
using Dets = vision_msgs::msg::Detection3DArray;

static const Isometry3d CONVEYOR_TOP = trans(-0.9, 0.0, 0.70);     // belt surface at the photo-eye, world frame

class Executive : public rclcpp::Node {
public:
  Executive() : Node("executive"), buf_(get_clock()), listener_(buf_), scene_(*this) {
    cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    capture_ = create_client<std_srvs::srv::Trigger>("camera/capture", rmw_qos_profile_services_default, cb_);
    pick_place_ = rclcpp_action::create_client<PickPlace>(this, "pick_place", cb_);
    index_out_ = create_client<std_srvs::srv::Trigger>("conveyor/index_out", rmw_qos_profile_services_default, cb_);
    feed_in_ = create_client<std_srvs::srv::Trigger>("conveyor/feed_in", rmw_qos_profile_services_default, cb_);
    rclcpp::SubscriptionOptions so; so.callback_group = cb_;
    sub_ = create_subscription<Dets>("detections", 10, [this](Dets::ConstSharedPtr m) { std::lock_guard<std::mutex> l(mtx_); frame_ = m; }, so);
    eye_sub_ = create_subscription<std_msgs::msg::Bool>("photo_eye", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { std::lock_guard<std::mutex> l(mtx_); eye_ = m->data; }, so);
    sku_sub_ = create_subscription<std_msgs::msg::String>("conveyor/sku", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) { std::lock_guard<std::mutex> l(mtx_); sku_at_eye_ = m->data; }, so);
    server_ = rclcpp_action::create_server<RunJob>(
      this, "run_job",
      [](const rclcpp_action::GoalUUID &, RunJob::Goal::ConstSharedPtr) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](std::shared_ptr<JobHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<JobHandle> gh) { std::thread([this, gh]() { execute(gh); }).detach(); },
      rcl_action_server_get_default_options(), cb_);
  }

private:
  // ---- waiting from the job thread: the executor keeps serving callbacks meanwhile ----
  bool capture() { return callTrigger(capture_, 15s).first; }
  /// A PLC-style command: the request is the output you set, the response is the done signal.
  std::pair<bool, std::string> callTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client, std::chrono::seconds timeout) {
    auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (fut.wait_for(timeout) != std::future_status::ready) return {false, "timeout"};
    const auto r = fut.get();
    return {r->success, r->message};
  }
  void clearFrame() { std::lock_guard<std::mutex> l(mtx_); frame_.reset(); }
  std::optional<Dets> waitFrame(std::chrono::milliseconds timeout) {     // checks at least once, even with 0 ms
    const auto t0 = std::chrono::steady_clock::now();
    do {
      { std::lock_guard<std::mutex> l(mtx_); if (frame_) return *frame_; }
      std::this_thread::sleep_for(50ms);
    } while (std::chrono::steady_clock::now() - t0 < timeout);
    return std::nullopt;
  }
  /// Sends a PickPlace goal and waits for it. Returns (success, message).
  std::pair<bool, std::string> runPick(const PickPlace::Goal & goal) {
    rclcpp_action::Client<PickPlace>::SendGoalOptions opts;
    opts.feedback_callback = [this](auto, PickPlace::Feedback::ConstSharedPtr f) { onStage(*f); };
    auto gh_fut = pick_place_->async_send_goal(goal, opts);
    if (gh_fut.wait_for(10s) != std::future_status::ready || !gh_fut.get()) return {false, "pick_place rejected the goal"};
    auto res_fut = pick_place_->async_get_result(gh_fut.get());
    if (res_fut.wait_for(180s) != std::future_status::ready) return {false, "pick_place timeout"};
    const auto r = res_fut.get();
    return {r.result && r.result->success, r.result ? r.result->message : "no result"};
  }
  void onStage(const PickPlace::Feedback & f) {
    RCLCPP_INFO(get_logger(), "   %s", f.stage.c_str());
    if (f.stage == "TRANSIT" && mode_ == "depalletize") {                  // Ch 18: the pallet is free once the box is lifted
      clearFrame();
      capture_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
  }

  // ---- the loop ----
  void execute(std::shared_ptr<JobHandle> gh) {
    auto fb = std::make_shared<RunJob::Feedback>();
    auto res = std::make_shared<RunJob::Result>();
    const auto & req = *gh->get_goal();
    const std::string mode = req.mode;
    mode_ = mode;
    std::string state = "CAPTURE";
    int moved = 0, retries = 0;
    Dets dets;
    std::optional<PickPlace::Goal> goal;
    if (mode == "palletize") {
      if (req.strategy == "mixed") plan_ = std::make_unique<MixedPlan>(); else plan_ = std::make_unique<SingleSkuPlan>(3);
      state = "FEED";
    }
    scene_.clear();
    while (true) {
      fb->state = state; fb->boxes_moved = moved; gh->publish_feedback(fb);
      RCLCPP_INFO(get_logger(), "[%d] %s", moved, state.c_str());
      if (gh->is_canceling()) { res->boxes_moved = moved; res->message = "cancelled"; gh->canceled(res); return; }

      if (state == "CAPTURE") {
        clearFrame();
        if (!capture()) { last_error_ = "camera timeout"; state = "RECOVER"; continue; }
        state = "DETECT";
      } else if (state == "DETECT") {
        const auto f = waitFrame(2s);
        if (!f) { last_error_ = "no detections"; state = "RECOVER"; continue; }
        dets = *f;
        if (dets.detections.empty() || (req.max_boxes >= 0 && moved >= req.max_boxes)) { state = "DONE"; continue; }
        state = "SELECT";
      } else if (state == "FEED") {
        if (req.max_boxes >= 0 && moved >= req.max_boxes) { state = "DONE"; continue; }
        const auto [ok, msg] = callTrigger(feed_in_, 30s);
        if (!ok) { last_error_ = msg; state = "RECOVER"; continue; }
        state = "SELECT";
      } else if (state == "SELECT") {
        goal = mode == "depalletize" ? planPick(dets) : planPlace();
        state = goal ? "PICK_PLACE" : "DONE";
      } else if (state == "PICK_PLACE") {
        const auto [ok, msg] = runPick(*goal);
        if (ok) {
          ++moved; retries = 0; retries_ = 0;
          if (mode == "depalletize") { callTrigger(index_out_, 15s); state = waitFrame(0ms) ? "DETECT" : "CAPTURE"; }   // Ch 18: the next frame is usually ready
          else { plan_->confirm(); state = "FEED"; }
        } else { last_error_ = msg; state = "RECOVER"; }
      } else if (state == "RECOVER") {
        ++retries; retries_ = retries;
        RCLCPP_ERROR(get_logger(), "recover (%d/3): %s", retries, last_error_.c_str());
        if (last_error_.find("still holding") != std::string::npos || retries > 3) { state = "FAILED"; continue; }
        state = mode == "depalletize" ? "CAPTURE" : "FEED";
      } else {                                                          // DONE or FAILED
        res->success = state == "DONE"; res->boxes_moved = moved; res->message = state;
        if (res->success) gh->succeed(res); else gh->abort(res);
        return;
      }
    }
  }

  /// Highest SAFE box first; SKU height overrides the detector's; skip grasps that risk a double pick.
  std::optional<PickPlace::Goal> planPick(const Dets & dets) {
    auto top = [](const vision_msgs::msg::Detection3D & d) { return d.bbox.center.position.z + d.bbox.size.z / 2; };
    auto sizeOf = [](const vision_msgs::msg::Detection3D & o) { return matchSku(o.bbox.size.x, o.bbox.size.y).second; };
    std::vector<const vision_msgs::msg::Detection3D *> order;
    for (const auto & d : dets.detections) order.push_back(&d);
    std::sort(order.begin(), order.end(), [&](const auto * a, const auto * b) { return top(*a) > top(*b); });   // highest top first
    for (const auto * dp : order) {
      const auto & d = *dp;
      const auto [sku, size] = matchSku(d.bbox.size.x, d.bbox.size.y);
      Isometry3d T_box = matFromPose(d.bbox.center);
      T_box.translation().z() = top(d) - size.z / 2;
      std::vector<Neighbor> others;
      for (const auto & o : dets.detections) if (o.id != d.id) others.push_back({matFromPose(o.bbox.center), sizeOf(o)});
      auto [risk, T_pick] = bestGrasp(T_box, size, others);
      if (risk > 0.1) { RCLCPP_WARN(get_logger(), "%s: double-pick risk %.2f, trying the next box", d.id.c_str(), risk); continue; }
      if (retries_ % 2) {                                                  // Ch 16: try the other yaw on a retry
        const Isometry3d alt = graspCandidates(T_box, size)[1];
        if (doublePickRisk(alt, others) <= 0.1) T_pick = alt;
      }
      scene_.sync(dets, sizeOf);
      const Isometry3d T_wp = tf2::transformToEigen(buf_.lookupTransform("world", "pallet_frame", tf2::TimePointZero, tf2::durationFromSec(1.0)));
      const Isometry3d T_place = CONVEYOR_TOP * trans(0.0, 0.0, size.z) * rotX(M_PI);   // TCP at box-top height over the eye
      PickPlace::Goal g;
      g.box_id = d.id;
      g.pick_pose = stamped(T_wp * T_pick);
      g.place_pose = stamped(T_place);
      g.box_size.x = size.x; g.box_size.y = size.y; g.box_size.z = size.z;
      g.approach_distance = 0.15 + 0.05 * retries_;                        // Ch 16: a higher approach on each retry
      RCLCPP_INFO(get_logger(), "%s: sku '%s' risk %.2f approach %.2f", d.id.c_str(), sku.c_str(), risk, g.approach_distance);
      return g;
    }
    return std::nullopt;                                                  // nothing safe: DONE, a person looks
  }

  /// Palletizing: the box at the eye goes to the next pattern slot.
  std::optional<PickPlace::Goal> planPlace() {
    std::string sku;
    { std::lock_guard<std::mutex> l(mtx_); if (!eye_ || !SKUS.count(sku_at_eye_)) return std::nullopt; sku = sku_at_eye_; }
    const BoxSize size = SKUS.at(sku);
    const auto T_target = plan_->target(size);
    if (!T_target) return std::nullopt;                                   // pallet full
    const Isometry3d T_belt_box = CONVEYOR_TOP * trans(0.0, 0.0, size.z / 2);   // guides center the box, yaw 0
    const auto [risk, T_pick] = bestGrasp(T_belt_box, size, {});
    const Isometry3d T_wp = tf2::transformToEigen(buf_.lookupTransform("world", "pallet_frame", tf2::TimePointZero, tf2::durationFromSec(1.0)));
    const Isometry3d T_place = T_wp * topDownTcp(*T_target, size.z);      // pad on the box top, pattern yaw
    scene_.clear();
    int n = 0;
    for (const auto & [T, s] : plan_->placed()) scene_.add("pal_" + std::to_string(n++), T, s);   // what we believe is on the pallet
    PickPlace::Goal g;
    g.box_id = "belt";
    g.pick_pose = stamped(T_pick);
    g.place_pose = stamped(T_place);
    g.box_size.x = size.x; g.box_size.y = size.y; g.box_size.z = size.z;
    return g;
  }

  rclcpp::CallbackGroup::SharedPtr cb_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_, index_out_, feed_in_;
  rclcpp_action::Client<PickPlace>::SharedPtr pick_place_;
  rclcpp::Subscription<Dets>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr eye_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sku_sub_;
  bool eye_ = false;
  std::string sku_at_eye_;                                 // both guarded by mtx_
  std::unique_ptr<Plan> plan_;
  rclcpp_action::Server<RunJob>::SharedPtr server_;
  tf2_ros::Buffer buf_;
  tf2_ros::TransformListener listener_;
  cell_executive::Scene scene_;
  std::mutex mtx_;
  Dets::ConstSharedPtr frame_;
  std::string last_error_;
  int retries_ = 0;                                        // Ch 16: varies approach distance and grasp on retry
  std::string mode_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Executive>();
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 4);
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
}
