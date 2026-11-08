// Ch 10: PointCloud2 -> pallet_frame points -> detect() -> Detection3DArray + debug markers.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cell_perception/heightmap.hpp>
#include <cell_planning/geometry.hpp>

using cell_planning::poseFromMat; using cell_planning::rotZ; using cell_planning::trans;

class BoxDetector : public rclcpp::Node {
public:
  BoxDetector() : Node("box_detector"), buf_(get_clock()), listener_(buf_) {
    p_.cell = declare_parameter("cell_m", 0.01);
    p_.layer_tol = declare_parameter("layer_tol_m", 0.03);
    p_.min_area = declare_parameter("min_area_m2", 0.015);
    p_.min_height = declare_parameter("min_height_m", 0.03);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("camera/points", rclcpp::SensorDataQoS(),
                                                               [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onCloud(*m); });
    pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("detections", 10);
    pub_dbg_ = create_publisher<visualization_msgs::msg::MarkerArray>("detections/markers", 10);
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & msg) {
    Eigen::Isometry3d T;
    try {
      T = tf2::transformToEigen(buf_.lookupTransform("pallet_frame", msg.header.frame_id, tf2::TimePointZero));
    } catch (const tf2::TransformException & e) { RCLCPP_WARN(get_logger(), "%s", e.what()); return; }

    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg.width * msg.height);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(msg, "x"); it != it.end(); ++it)
      pts.push_back(T * Eigen::Vector3d(it[0], it[1], it[2]));          // x, y, z are adjacent floats
    const auto dets = cell_perception::detect(pts, p_);

    vision_msgs::msg::Detection3DArray arr;
    arr.header.stamp = msg.header.stamp; arr.header.frame_id = "pallet_frame";
    visualization_msgs::msg::MarkerArray mk;
    visualization_msgs::msg::Marker clear; clear.action = visualization_msgs::msg::Marker::DELETEALL; mk.markers.push_back(clear);
    for (size_t i = 0; i < dets.size(); ++i) {
      const auto & d = dets[i];
      vision_msgs::msg::Detection3D det;
      det.header = arr.header; det.id = "det_" + std::to_string(i);
      det.bbox.center = poseFromMat(trans(d.x, d.y, d.z_top - d.h / 2) * rotZ(d.yaw));
      det.bbox.size.x = d.sx; det.bbox.size.y = d.sy; det.bbox.size.z = d.h;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = "box"; hyp.hypothesis.score = std::min(1.0, d.fill);
      det.results.push_back(hyp);
      arr.detections.push_back(det);

      visualization_msgs::msg::Marker m;
      m.header = arr.header; m.ns = "det"; m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CUBE; m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = det.bbox.center; m.scale = det.bbox.size; m.color.g = 1.0f; m.color.a = 0.4f;
      mk.markers.push_back(m);
    }
    pub_->publish(arr); pub_dbg_->publish(mk);
  }

  cell_perception::Params p_;
  tf2_ros::Buffer buf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dbg_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxDetector>());
  rclcpp::shutdown();
}
