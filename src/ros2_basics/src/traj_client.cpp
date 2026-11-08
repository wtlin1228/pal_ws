// Ch 5 checkpoint: send the same two-point trajectory from C++ and print the result's error code.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

using FJT = control_msgs::action::FollowJointTrajectory;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_client");
  auto client = rclcpp_action::create_client<FJT>(node, "/arm_controller/follow_joint_trajectory");
  client->wait_for_action_server();

  FJT::Goal goal;
  goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  const std::vector<std::pair<std::vector<double>, int>> points = {{{0.8, -1.3, 1.4, -1.67, -1.57, 0.0}, 3}, {{0.0, -1.57, 1.57, -1.57, -1.57, 0.0}, 6}};
  for (const auto & [positions, seconds] : points) {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = positions;
    p.time_from_start.sec = seconds;
    goal.trajectory.points.push_back(p);
  }
  auto gh_fut = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, gh_fut) != rclcpp::FutureReturnCode::SUCCESS || !gh_fut.get()) {
    RCLCPP_ERROR(node->get_logger(), "goal rejected"); return 1;
  }
  auto res_fut = client->async_get_result(gh_fut.get());
  rclcpp::spin_until_future_complete(node, res_fut);
  RCLCPP_INFO(node->get_logger(), "error_code %d (0 = SUCCESSFUL)", res_fut.get().result->error_code);
  rclcpp::shutdown();
}
