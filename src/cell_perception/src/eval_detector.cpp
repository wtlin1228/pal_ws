// Ch 10 checkpoint: score /detections against camera/ground_truth. Evaluation only; never used for control.
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <cell_interfaces/msg/box_array.hpp>
#include <cell_planning/geometry.hpp>

using namespace cell_planning;

class EvalDetector : public rclcpp::Node {
public:
  EvalDetector() : Node("eval_detector"), buf_(get_clock()), listener_(buf_) {
    truth_sub_ = create_subscription<cell_interfaces::msg::BoxArray>("camera/ground_truth", 10,
      [this](cell_interfaces::msg::BoxArray::ConstSharedPtr m) { truth_ = m; });
    det_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>("detections", 10,
      [this](vision_msgs::msg::Detection3DArray::ConstSharedPtr m) { onDets(*m); });
  }

private:
  struct Truth { Eigen::Vector3d top; double sx, sy, yaw; };

  void onDets(const vision_msgs::msg::Detection3DArray & msg) {
    if (!truth_ || truth_->boxes.empty()) return;
    Isometry3d T_pw;
    try { T_pw = tf2::transformToEigen(buf_.lookupTransform("pallet_frame", "world", tf2::TimePointZero)); }
    catch (const tf2::TransformException &) { return; }
    std::vector<Truth> truth;
    for (const auto & b : truth_->boxes) {                         // truth boxes are in world; compare in pallet_frame
      const Isometry3d T = T_pw * matFromPose(b.pose.pose);
      truth.push_back({T.translation() + Eigen::Vector3d(0, 0, b.size.z / 2), std::max(b.size.x, b.size.y), std::min(b.size.x, b.size.y), yawOf(T)});
    }
    double e_xy = 0, e_size = 0, e_yaw = 0;
    for (const auto & d : msg.detections) {
      const Isometry3d T = matFromPose(d.bbox.center);
      const Eigen::Vector3d top = T.translation() + Eigen::Vector3d(0, 0, d.bbox.size.z / 2);
      const auto & t = *std::min_element(truth.begin(), truth.end(), [&](const Truth & a, const Truth & b) {
        return (a.top.head<2>() - top.head<2>()).norm() < (b.top.head<2>() - top.head<2>()).norm(); });
      e_xy += (t.top.head<2>() - top.head<2>()).norm();
      e_size += std::max(std::abs(d.bbox.size.x - t.sx), std::abs(d.bbox.size.y - t.sy));
      e_yaw += std::abs(std::remainder(yawOf(T) - t.yaw, M_PI));   // the long side is symmetric under 180 deg
    }
    const double n = std::max<std::size_t>(1, msg.detections.size());
    RCLCPP_INFO(get_logger(), "%zu detections vs %zu truth boxes: |dxy| %.1f mm  |dsize| %.1f mm  |dyaw| %.1f deg",
                msg.detections.size(), truth.size(), 1e3 * e_xy / n, 1e3 * e_size / n, 180.0 / M_PI * e_yaw / n);
  }

  tf2_ros::Buffer buf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<cell_interfaces::msg::BoxArray>::SharedPtr truth_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det_sub_;
  cell_interfaces::msg::BoxArray::ConstSharedPtr truth_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvalDetector>());
  rclcpp::shutdown();
}
