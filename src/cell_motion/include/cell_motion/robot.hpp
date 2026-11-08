// Ch 7: every motion the cell needs, expressed as TCP poses, on top of MoveGroupInterface.
#pragma once
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cell_planning/geometry.hpp>

namespace cell_motion {

/// Every motion the cell needs, expressed as TCP poses in `world`. One instance per node.
class Robot {
public:
  using MoveGroup = moveit::planning_interface::MoveGroupInterface;

  explicit Robot(const rclcpp::Node::SharedPtr & node)
  : node_(node), mg_(node, "arm"), buf_(node->get_clock()), listener_(buf_) {
    mg_.setEndEffectorLink("tcp");
    mg_.setPoseReferenceFrame("world");
    mg_.setMaxVelocityScalingFactor(0.5);           // scaling 0..1 of the joint_limits.yaml values
    mg_.setMaxAccelerationScalingFactor(0.5);
    mg_.setPlanningTime(2.0);
    mg_.setNumPlanningAttempts(5);
    setPlanner("ompl", "RRTConnectkConfigDefault");
  }

  void setPlanner(const std::string & pipeline, const std::string & planner) {
    pipeline_ = pipeline; planner_ = planner;
    mg_.setPlanningPipelineId(pipeline);            // "ompl" | "pilz_industrial_motion_planner"
    mg_.setPlannerId(planner);                      // "RRTConnectkConfigDefault" | "PTP" | "LIN"
  }

  bool goHome() { mg_.setNamedTarget("home"); return planAndExecute(); }

  /// Joint-space motion with the current planner; path shape is up to the planner.
  bool moveTo(const geometry_msgs::msg::PoseStamped & pose) { mg_.setPoseTarget(pose); return planAndExecute(); }

  /// Straight TCP line. Default: MoveIt's Cartesian-path service (IK per 5 mm step, collisions checked).
  /// use_pilz: the industrial LIN planner with the limits from pilz_cartesian_limits.yaml.
  bool moveLinear(const geometry_msgs::msg::PoseStamped & pose, bool use_pilz = false) {
    if (use_pilz) {
      const auto previous = std::make_pair(pipeline_, planner_);
      setPlanner("pilz_industrial_motion_planner", "LIN");
      const bool ok = moveTo(pose);
      setPlanner(previous.first, previous.second);
      return ok;
    }
    moveit_msgs::msg::RobotTrajectory traj;
    const double fraction = mg_.computeCartesianPath({pose.pose}, 0.005, traj, true);
    if (fraction < 0.95) {                          // refuse paths that stop short
      RCLCPP_WARN(node_->get_logger(), "cartesian path only %.0f %% feasible", 100 * fraction);
      return false;
    }
    return mg_.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  std::vector<double> jointPositions() { return mg_.getCurrentJointValues(); }

  /// Ch 7 checkpoint: the TCP pose in world from TF, not from MoveIt.
  Eigen::Isometry3d tcpPose() {
    return cell_planning::matFromTf(
      buf_.lookupTransform("world", "tcp", tf2::TimePointZero, tf2::durationFromSec(1.0)).transform);
  }

  MoveGroup & mg() { return mg_; }

private:
  bool planAndExecute() {
    MoveGroup::Plan plan;
    if (mg_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    return mg_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  MoveGroup mg_;
  tf2_ros::Buffer buf_;
  tf2_ros::TransformListener listener_;
  std::string pipeline_, planner_;
};

}  // namespace cell_motion
