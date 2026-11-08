// Ch 12: draw layers A and B as RViz markers in pallet_frame, with the placement order as text.
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cell_planning/patterns.hpp>

using namespace cell_planning;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("show_pattern");
  auto pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("pattern/markers", rclcpp::QoS(1).transient_local());
  visualization_msgs::msg::MarkerArray arr;
  int id = 0;
  for (int L : {0, 1}) {
    const auto & layer = L ? layerB() : layerA();
    int order = 1;
    for (const auto & p : placeOrder(layer)) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "pallet_frame"; m.ns = "pattern"; m.id = id++;
      m.type = m.CUBE; m.action = m.ADD;
      m.pose = poseFromMat(boxPose(p, L));
      m.scale.x = SKU_A.x; m.scale.y = SKU_A.y; m.scale.z = SKU_A.z;
      m.color.r = 0.91f; m.color.g = 0.38f; m.color.b = 0.15f; m.color.a = 0.8f;
      arr.markers.push_back(m);
      visualization_msgs::msg::Marker t = m;
      t.id = id++; t.type = t.TEXT_VIEW_FACING; t.text = std::to_string(order++);
      t.pose.position.z += SKU_A.z / 2 + 0.02; t.scale.z = 0.06; t.color.r = t.color.g = t.color.b = 1.0f;
      arr.markers.push_back(t);
    }
  }
  pub->publish(arr);                                  // latched: RViz gets it whenever it subscribes
  RCLCPP_INFO(node->get_logger(), "layers A (deck) and B (one layer up) published on pattern/markers");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
