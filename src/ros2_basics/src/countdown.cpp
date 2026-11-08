// Ch 1: an action server with feedback and cancel. Fibonacci is a placeholder type until Ch 2.
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

class Countdown : public rclcpp::Node {
public:
  Countdown() : Node("countdown") {
    server_ = rclcpp_action::create_server<Fibonacci>(
      this, "countdown",
      [](const rclcpp_action::GoalUUID &, Fibonacci::Goal::ConstSharedPtr) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<GoalHandle> gh) {
        std::thread([this, gh]() { execute(gh); }).detach();   // never block the executor with a long goal
      });
  }

private:
  void execute(std::shared_ptr<GoalHandle> gh) {
    const int n = gh->get_goal()->order;
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();
    for (int i = 0; i < n; ++i) {
      if (gh->is_canceling()) { gh->canceled(result); return; }
      feedback->sequence = {n - i};
      gh->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    result->sequence = {0};
    gh->succeed(result);
  }
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Countdown>());           // feedback and cancel arrive while execute() sleeps in its thread
  rclcpp::shutdown();
}
