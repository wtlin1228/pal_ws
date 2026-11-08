// Ch 11: the eight-stage pick-and-place as a ROS 2 action, keeping MoveIt's planning scene honest.
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <cell_interfaces/action/pick_place.hpp>
#include <cell_interfaces/srv/set_suction.hpp>
#include <cell_motion/robot.hpp>

using PickPlace = cell_interfaces::action::PickPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
using SetSuction = cell_interfaces::srv::SetSuction;
using cell_planning::matFromPose; using cell_planning::poseFromMat; using cell_planning::trans;

class PickPlaceServer : public rclcpp::Node {
public:
  explicit PickPlaceServer(const rclcpp::NodeOptions & options) : Node("pick_place_server", options) {}

  void init() {                                              // needs shared_from_this(): not callable from the constructor
    robot_ = std::make_unique<cell_motion::Robot>(shared_from_this());
    cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    suction_ = create_client<SetSuction>("gripper/set_suction", rmw_qos_profile_services_default, cb_);
    server_ = rclcpp_action::create_server<PickPlace>(
      this, "pick_place",
      [](const rclcpp_action::GoalUUID &, PickPlace::Goal::ConstSharedPtr) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<GoalHandle> gh) { std::thread([this, gh]() { execute(gh); }).detach(); },
      rcl_action_server_get_default_options(), cb_);
  }

private:
  /// Shift along the TCP's own z. Negative = away from the box (up, for a top-down grasp).
  static geometry_msgs::msg::PoseStamped offset(const geometry_msgs::msg::PoseStamped & pose, double dz) {
    geometry_msgs::msg::PoseStamped out;
    out.header = pose.header;
    out.pose = poseFromMat(matFromPose(pose.pose) * trans(0.0, 0.0, dz));
    return out;
  }

  bool suction(bool on) {
    auto req = std::make_shared<SetSuction::Request>();
    req->on = on;
    auto fut = suction_->async_send_request(req);              // the executor thread serves the reply
    if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) { last_msg_ = "gripper timeout"; return false; }
    const auto res = fut.get();
    last_msg_ = res->message;
    return res->success;
  }

  void attachBox(const PickPlace::Goal & g) {
    moveit_msgs::msg::CollisionObject co;                      // the box as seen from the TCP: its top face at z = 0
    co.id = g.box_id;
    co.header.frame_id = "tcp";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {g.box_size.x, g.box_size.y, g.box_size.z};
    co.primitives.push_back(box);
    co.primitive_poses.push_back(poseFromMat(trans(0.0, 0.0, g.box_size.z / 2)));
    co.operation = co.ADD;
    scene_.applyCollisionObject(co);
    robot_->mg().attachObject(g.box_id, "tcp", {"suction_pad", "gripper_body"});   // touch_links: the pad may touch it
  }

  void execute(std::shared_ptr<GoalHandle> gh) {
    std::lock_guard<std::mutex> lock(mutex_);                  // one motion at a time
    const auto g = gh->get_goal();
    auto fb = std::make_shared<PickPlace::Feedback>();
    auto res = std::make_shared<PickPlace::Result>();
    const double d = g->approach_distance;
    auto stage = [&](const char * name) { fb->stage = name; gh->publish_feedback(fb); RCLCPP_INFO(get_logger(), "%s: %s", g->box_id.c_str(), name); };
    auto fail = [&](const std::string & msg) { res->success = false; res->message = msg; RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); gh->abort(res); };

    stage("APPROACH");
    scene_.removeCollisionObjects({g->box_id});                // fact 2: the planner cannot plan contact
    if (!robot_->moveTo(offset(g->pick_pose, -d)))      return fail("approach: no plan");
    stage("DESCEND");
    if (!robot_->moveLinear(g->pick_pose))              return fail("descend failed");
    stage("GRIP");
    if (!suction(true)) { robot_->moveLinear(offset(g->pick_pose, -d)); return fail("no vacuum: " + last_msg_); }
    attachBox(*g);
    stage("LIFT");
    if (!robot_->moveLinear(offset(g->pick_pose, -d)))  return fail("lift failed, still holding");
    stage("TRANSIT");
    if (!robot_->moveTo(offset(g->place_pose, -d)))     return fail("transit: no plan, still holding");
    stage("PLACE");
    if (!robot_->moveLinear(g->place_pose))             return fail("place failed, still holding");
    stage("RELEASE");
    suction(false);
    robot_->mg().detachObject(g->box_id);
    scene_.removeCollisionObjects({g->box_id});
    stage("RETREAT");
    if (!robot_->moveLinear(offset(g->place_pose, -d))) return fail("retreat failed");
    res->success = true; res->message = "ok";
    gh->succeed(res);
  }

  std::unique_ptr<cell_motion::Robot> robot_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp::CallbackGroup::SharedPtr cb_;
  rclcpp::Client<SetSuction>::SharedPtr suction_;
  rclcpp_action::Server<PickPlace>::SharedPtr server_;
  std::mutex mutex_;
  std::string last_msg_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceServer>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 4);
  ex.add_node(node);
  std::thread spinner([&ex]() { ex.spin(); });
  node->init();                                              // MoveGroupInterface needs the executor running
  spinner.join();
  rclcpp::shutdown();
}
