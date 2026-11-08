// Ch 7: a sequential program next to a spinning executor.
#include <chrono>
#include <cmath>
#include <thread>
#include <cell_motion/robot.hpp>
#include <cell_motion/stamped.hpp>

using cell_motion::stamped;
using cell_planning::rotX;
using cell_planning::trans;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_demo",
                                        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spinner([&ex]() { ex.spin(); });            // callbacks run here; main() stays a plain program

  cell_motion::Robot robot(node);                         // connects to move_group (robot model from the topic)
  const auto above = trans(0.0, 0.75, 0.144 + 0.45) * rotX(M_PI);   // 45 cm over the deck, pad facing down

  bool ok = robot.goHome();
  const auto t0 = std::chrono::steady_clock::now();
  ok = ok && robot.moveTo(stamped(above));                            // free-space move
  RCLCPP_INFO(node->get_logger(), "OMPL free-space move: %.2f s",
              std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
  ok = ok && robot.moveLinear(stamped(above * trans(0, 0, 0.15)));   // descend: +z in the TCP frame is DOWN
  ok = ok && robot.moveLinear(stamped(above));                       // retreat
  if (ok) {
    const double err = (robot.tcpPose().translation() - above.translation()).norm();
    RCLCPP_INFO(node->get_logger(), "TF vs goal: %.2f mm (the mock plugin is exact)", 1e3 * err);
    robot.setPlanner("pilz_industrial_motion_planner", "PTP");
    ok = robot.goHome();
  }
  RCLCPP_INFO(node->get_logger(), ok ? "done" : "a move failed");
  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
