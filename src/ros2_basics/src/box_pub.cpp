// Ch 2 checkpoint: publish a BoxArray with two boxes every second.
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <cell_interfaces/msg/box_array.hpp>

class BoxPub : public rclcpp::Node {
public:
  BoxPub() : Node("box_pub") {
    pub_ = create_publisher<cell_interfaces::msg::BoxArray>("boxes", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { tick(); });
  }

private:
  void tick() {
    cell_interfaces::msg::BoxArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = "pallet_frame";
    int i = 0;
    for (double x : {-0.16, 0.16}) {
      cell_interfaces::msg::Box b;
      b.id = "box_00" + std::to_string(i++);
      b.sku = "A";
      b.size.x = 0.30; b.size.y = 0.18; b.size.z = 0.15;
      b.location = "pallet";
      b.pose.header = msg.header;
      b.pose.pose.position.x = x; b.pose.pose.position.z = 0.075;
      b.pose.pose.orientation.w = 1.0;
      msg.boxes.push_back(b);
    }
    pub_->publish(msg);
  }
  rclcpp::Publisher<cell_interfaces::msg::BoxArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxPub>());
  rclcpp::shutdown();
}
