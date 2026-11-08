// Ch 4: a static frame (a box on the pallet), a dynamic frame (a box riding the belt) and a lookup.
#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TfDemo : public rclcpp::Node {
public:
  TfDemo() : Node("tf_demo"), buf_(get_clock()), listener_(buf_), static_br_(this), br_(this) {
    geometry_msgs::msg::TransformStamped t;                 // a box that never moves: static
    t.header.stamp = now();
    t.header.frame_id = "pallet_frame";
    t.child_frame_id = "box_1";
    t.transform.translation.x = -0.25; t.transform.translation.y = 0.15; t.transform.translation.z = 0.075;
    t.transform.rotation.w = 1.0;
    static_br_.sendTransform(t);

    belt_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { beltTick(); });   // dynamic, 20 Hz
    report_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

private:
  void beltTick() {
    y_ = std::min(y_ + 0.01, 0.0);                          // 0.2 m/s, stops at the photo-eye
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "conveyor_frame";
    t.child_frame_id = "box_belt";
    t.transform.translation.y = y_; t.transform.translation.z = 0.075;
    t.transform.rotation.w = 1.0;
    br_.sendTransform(t);
  }
  void report() {
    try {
      const auto tf = buf_.lookupTransform("pallet_frame", "tcp", tf2::TimePointZero, tf2::durationFromSec(0.5));
      const auto & p = tf.transform.translation;
      RCLCPP_INFO(get_logger(), "tcp in pallet_frame: x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s", e.what());
    }
  }
  tf2_ros::Buffer buf_;                         // filled from /tf and /tf_static by the listener
  tf2_ros::TransformListener listener_;         // owns a spinning thread of its own
  tf2_ros::StaticTransformBroadcaster static_br_;
  tf2_ros::TransformBroadcaster br_;
  rclcpp::TimerBase::SharedPtr belt_timer_, report_timer_;
  double y_{-0.7};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfDemo>());
  rclcpp::shutdown();
}
