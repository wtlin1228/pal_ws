// Ch 13: keeps move_group's world in sync with what the executive believes, via the /collision_object topic.
#pragma once
#include <set>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <cell_planning/geometry.hpp>
#include <cell_planning/grasps.hpp>

namespace cell_executive {

/// Keeps move_group's world in sync with what the executive believes, via the /collision_object topic.
class Scene {
public:
  explicit Scene(rclcpp::Node & node, std::string frame = "pallet_frame") : node_(node), frame_(std::move(frame)) {
    pub_ = node.create_publisher<moveit_msgs::msg::CollisionObject>("collision_object", 10);
  }

  void add(const std::string & id, const Eigen::Isometry3d & T, const cell_planning::BoxSize & s, const std::string & frame = "") {
    auto co = msg(id, moveit_msgs::msg::CollisionObject::ADD, frame);
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {s.x, s.y, s.z};
    co.primitives.push_back(box);
    co.primitive_poses.push_back(cell_planning::poseFromMat(T));
    pub_->publish(co);
    ids_.insert(id);
  }

  void remove(const std::string & id) { pub_->publish(msg(id, moveit_msgs::msg::CollisionObject::REMOVE)); ids_.erase(id); }

  void clear() { for (const auto & id : std::set<std::string>(ids_)) remove(id); }

  /// One box per detection plus a solid envelope for everything below the top layer.
  template <typename SizeOf>
  void sync(const vision_msgs::msg::Detection3DArray & dets, SizeOf sizeOf) {
    for (const auto & id : std::set<std::string>(ids_))
      if (id.rfind("det_", 0) == 0 || id == "stack") remove(id);
    double z_top = 0.0, tallest = 0.0;
    for (const auto & d : dets.detections) {
      const cell_planning::BoxSize s = sizeOf(d);
      const double top = d.bbox.center.position.z + d.bbox.size.z / 2;
      Eigen::Isometry3d T = cell_planning::matFromPose(d.bbox.center);
      T.translation().z() = top - s.z / 2;                               // re-center with the SKU height
      add(d.id, T, s);
      z_top = std::max(z_top, top);
      tallest = std::max(tallest, s.z);
    }
    const double below = z_top - tallest;
    if (below > 0.01)                                                    // we cannot see under the top layer
      add("stack", cell_planning::trans(0.0, 0.0, below / 2), {0.8, 0.6, below});
  }

private:
  moveit_msgs::msg::CollisionObject msg(const std::string & id, int8_t op, const std::string & frame = "") {
    moveit_msgs::msg::CollisionObject co;
    co.id = id;
    co.operation = op;
    co.header.frame_id = frame.empty() ? frame_ : frame;
    co.header.stamp = node_.now();
    return co;
  }

  rclcpp::Node & node_;
  std::string frame_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr pub_;
  std::set<std::string> ids_;
};

}  // namespace cell_executive
