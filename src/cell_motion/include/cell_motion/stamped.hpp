// A PoseStamped from a 4x4 pose, shared by every demo and server.
#pragma once
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cell_planning/geometry.hpp>

namespace cell_motion {

inline geometry_msgs::msg::PoseStamped stamped(const Eigen::Isometry3d & T, const std::string & frame = "world") {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose = cell_planning::poseFromMat(T);
  return p;
}

}  // namespace cell_motion
