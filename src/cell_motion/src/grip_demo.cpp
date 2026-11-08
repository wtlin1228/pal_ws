// Ch 8: first pick. Reads the box pose from world/get_boxes, which is allowed ONLY here, as a test of the twin.
#include <cmath>
#include <thread>
#include <cell_interfaces/srv/get_boxes.hpp>
#include <cell_interfaces/srv/set_suction.hpp>
#include <cell_motion/robot.hpp>
#include <cell_motion/stamped.hpp>

using namespace cell_planning;
using cell_motion::stamped;
using GetBoxes = cell_interfaces::srv::GetBoxes;
using SetSuction = cell_interfaces::srv::SetSuction;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("grip_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spinner([&ex]() { ex.spin(); });
  cell_motion::Robot robot(node);
  auto get_boxes = node->create_client<GetBoxes>("world/get_boxes");
  auto suction = node->create_client<SetSuction>("gripper/set_suction");
  get_boxes->wait_for_service(); suction->wait_for_service();
  auto setSuction = [&](bool on) { auto r = std::make_shared<SetSuction::Request>(); r->on = on; return suction->async_send_request(r).get()->success; };

  auto req = std::make_shared<GetBoxes::Request>(); req->location = "pallet";
  const auto boxes = get_boxes->async_send_request(req).get()->boxes;          // test-only shortcut
  if (boxes.empty()) { RCLCPP_ERROR(node->get_logger(), "no boxes on the pallet"); rclcpp::shutdown(); spinner.join(); return 1; }
  const auto & box = boxes.front();
  const Isometry3d T_grasp = topDownTcp(matFromPose(box.pose.pose), box.size.z);  // TCP on the top face, pad down
  const Isometry3d T_above = T_grasp * trans(0, 0, -0.15);                         // 15 cm up (TCP z points down)
  const Isometry3d T_drop = trans(-0.9, 0.0, 0.70 + box.size.z) * rotX(M_PI);      // pad at box-top height on the belt

  bool ok = robot.goHome()
    && robot.moveTo(stamped(T_above)) && robot.moveLinear(stamped(T_grasp))
    && setSuction(true)
    && robot.moveLinear(stamped(T_above))
    && robot.moveTo(stamped(T_drop * trans(0, 0, -0.15))) && robot.moveLinear(stamped(T_drop));
  setSuction(false);
  ok = ok && robot.moveLinear(stamped(T_drop * trans(0, 0, -0.15))) && robot.goHome();
  RCLCPP_INFO(node->get_logger(), ok ? "done" : "a step failed");
  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
