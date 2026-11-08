// Ch 1: a service server.
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Suction : public rclcpp::Node {
public:
  Suction() : Node("suction") {
    srv_ = create_service<std_srvs::srv::SetBool>(
      "set_suction",
      [this](std_srvs::srv::SetBool::Request::ConstSharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        on_ = req->data;
        res->success = true;
        res->message = on_ ? "suction ON" : "suction OFF";
      });
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  bool on_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Suction>());
  rclcpp::shutdown();
}
