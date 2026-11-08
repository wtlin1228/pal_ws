// Ch 1: a publisher with a parameter; the parameter callback (checkpoint) restarts the timer.
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class PhotoEye : public rclcpp::Node {
public:
  PhotoEye() : Node("photo_eye") {
    declare_parameter("period_s", 2.0);                    // how often a box "arrives"
    pub_ = create_publisher<std_msgs::msg::Bool>("photo_eye", 10);
    startTimer(get_parameter("period_s").as_double());
    // checkpoint: without this callback `ros2 param set` changes the stored value and nothing else
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == "period_s") {
          if (p.as_double() <= 0.0) { result.successful = false; result.reason = "period_s must be > 0"; return result; }
          startTimer(p.as_double());
        }
      }
      return result;
    });
  }

private:
  void startTimer(double period_s) {
    timer_ = create_wall_timer(std::chrono::duration<double>(period_s), [this]() { tick(); });
  }
  void tick() {
    blocked_ = !blocked_;
    std_msgs::msg::Bool msg;
    msg.data = blocked_;
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "photo_eye blocked=%d", blocked_);
  }
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  bool blocked_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PhotoEye>());            // runs timers and callbacks until Ctrl-C
  rclcpp::shutdown();
}
