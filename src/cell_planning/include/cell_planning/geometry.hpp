#pragma once
#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace cell_planning {

using Eigen::Isometry3d;

inline Isometry3d matFromPose(const geometry_msgs::msg::Pose & p) { Isometry3d T; tf2::fromMsg(p, T); return T; }
inline Isometry3d matFromTf(const geometry_msgs::msg::Transform & t) { return tf2::transformToEigen(t); }
inline geometry_msgs::msg::Pose poseFromMat(const Isometry3d & T) { return tf2::toMsg(T); }

inline Isometry3d trans(double x = 0.0, double y = 0.0, double z = 0.0) { return Isometry3d(Eigen::Translation3d(x, y, z)); }
inline Isometry3d rotX(double a) { return Isometry3d(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX())); }
inline Isometry3d rotZ(double a) { return Isometry3d(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ())); }
inline double yawOf(const Isometry3d & T) { return std::atan2(T(1, 0), T(0, 0)); }

/// TCP pose for a suction grasp: at the box's top-face center, TCP z pointing into the box.
/// tcp z points out of the pad (Ch 3), so flip the box frame by pi about x.
inline Isometry3d topDownTcp(const Isometry3d & T_frame_box, double size_z) {
  return T_frame_box * trans(0.0, 0.0, size_z / 2) * rotX(M_PI);
}

}  // namespace cell_planning
