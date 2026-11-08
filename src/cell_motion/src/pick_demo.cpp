// Ch 11: the first vision-guided pick. One capture, the highest box, a grasp, one PickPlace goal.
#include <algorithm>
#include <chrono>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cell_interfaces/action/pick_place.hpp>
#include <cell_planning/geometry.hpp>
#include <cell_planning/grasps.hpp>
#include <cell_motion/stamped.hpp>

using namespace cell_planning;
using cell_motion::stamped;
using PickPlace = cell_interfaces::action::PickPlace;
using Dets = vision_msgs::msg::Detection3DArray;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_demo");
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread spinner([&ex]() { ex.spin(); });                // callbacks run here; main() stays sequential
  tf2_ros::Buffer buf(node->get_clock());
  tf2_ros::TransformListener listener(buf);

  std::mutex mtx; Dets::ConstSharedPtr frame;
  auto sub = node->create_subscription<Dets>("detections", 10, [&](Dets::ConstSharedPtr m) { std::lock_guard<std::mutex> l(mtx); frame = m; });
  auto capture = node->create_client<std_srvs::srv::Trigger>("camera/capture");
  auto client = rclcpp_action::create_client<PickPlace>(node, "pick_place");
  capture->wait_for_service(); client->wait_for_action_server();

  capture->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()).get();   // one capture
  std::this_thread::sleep_for(std::chrono::milliseconds(500));                              // -> one detection frame
  Dets dets;
  { std::lock_guard<std::mutex> l(mtx); if (!frame) { RCLCPP_ERROR(node->get_logger(), "no detections"); rclcpp::shutdown(); spinner.join(); return 1; } dets = *frame; }

  auto top = [](const vision_msgs::msg::Detection3D & d) { return d.bbox.center.position.z + d.bbox.size.z / 2; };
  const auto & d = *std::max_element(dets.detections.begin(), dets.detections.end(),
                                     [&](const auto & a, const auto & b) { return top(a) < top(b); });   // highest top face first
  const BoxSize size{d.bbox.size.x, d.bbox.size.y, d.bbox.size.z};
  std::vector<Neighbor> others;
  for (const auto & o : dets.detections)
    if (o.id != d.id) others.push_back({matFromPose(o.bbox.center), BoxSize{o.bbox.size.x, o.bbox.size.y, o.bbox.size.z}});
  const auto [risk, T_pick] = bestGrasp(matFromPose(d.bbox.center), size, others);
  RCLCPP_INFO(node->get_logger(), "picking %s, double-pick risk %.2f", d.id.c_str(), risk);
  const Isometry3d T_wp = tf2::transformToEigen(buf.lookupTransform("world", "pallet_frame", tf2::TimePointZero, tf2::durationFromSec(1.0)));

  PickPlace::Goal goal;
  goal.box_id = d.id;
  goal.box_size = d.bbox.size;
  goal.pick_pose = stamped(T_wp * T_pick);
  goal.place_pose = stamped(trans(-0.9, 0.0, 0.70 + size.z) * rotX(M_PI));
  rclcpp_action::Client<PickPlace>::SendGoalOptions opts;
  opts.feedback_callback = [&](auto, PickPlace::Feedback::ConstSharedPtr f) { RCLCPP_INFO(node->get_logger(), "stage %s", f->stage.c_str()); };
  auto gh = client->async_send_goal(goal, opts).get();        // blocks this thread only; the executor thread keeps spinning
  auto result = client->async_get_result(gh).get();
  RCLCPP_INFO(node->get_logger(), "result %d %s", result.result->success, result.result->message.c_str());
  rclcpp::shutdown();
  spinner.join();
}
