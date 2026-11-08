// Ch 13: the depalletizing job as an explicit state machine behind a RunJob action.
// Beliefs only: the last detection frame, the SKU table, and what it placed where. It never reads world_model.
#include <algorithm>
#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cell_interfaces/action/pick_place.hpp>
#include <cell_interfaces/action/run_job.hpp>
#include <cell_planning/geometry.hpp>
#include <cell_planning/grasps.hpp>
#include <cell_planning/skus.hpp>
#include <cell_executive/scene.hpp>
#include <cell_motion/stamped.hpp>

using namespace std::chrono_literals;
using namespace cell_planning;
using cell_motion::stamped;
using PickPlace = cell_interfaces::action::PickPlace;
using RunJob = cell_interfaces::action::RunJob;
using JobHandle = rclcpp_action::ServerGoalHandle<RunJob>;
using Dets = vision_msgs::msg::Detection3DArray;

static const Isometry3d CONVEYOR_TOP = trans(-0.9, 0.0, 0.70);     // belt surface at the photo-eye, world frame
static const std::vector<double> DROP_SLOTS = {0.0, 0.4, -0.4};     // y offsets along the belt; Ch 14 deletes these

class Executive : public rclcpp::Node {
public:
  Executive() : Node("executive"), buf_(get_clock()), listener_(buf_), scene_(*this) {
    cb_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    capture_ = create_client<std_srvs::srv::Trigger>("camera/capture", rmw_qos_profile_services_default, cb_);
    pick_place_ = rclcpp_action::create_client<PickPlace>(this, "pick_place", cb_);
    rclcpp::SubscriptionOptions so; so.callback_group = cb_;
    sub_ = create_subscription<Dets>("detections", 10, [this](Dets::ConstSharedPtr m) { std::lock_guard<std::mutex> l(mtx_); frame_ = m; }, so);
    server_ = rclcpp_action::create_server<RunJob>(
      this, "run_job",
      [](const rclcpp_action::GoalUUID &, RunJob::Goal::ConstSharedPtr) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](std::shared_ptr<JobHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<JobHandle> gh) { std::thread([this, gh]() { execute(gh); }).detach(); },
      rcl_action_server_get_default_options(), cb_);
    for (double s : DROP_SLOTS) placed_[s] = 0;                        // belief: boxes per slot
  }

private:
  // ---- waiting from the job thread: the executor keeps serving callbacks meanwhile ----
  bool capture() {
    auto fut = capture_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    return fut.wait_for(15s) == std::future_status::ready;
  }
  void clearFrame() { std::lock_guard<std::mutex> l(mtx_); frame_.reset(); }
  std::optional<Dets> waitFrame(std::chrono::milliseconds timeout) {     // checks at least once, even with 0 ms
    const auto t0 = std::chrono::steady_clock::now();
    do {
      { std::lock_guard<std::mutex> l(mtx_); if (frame_) return *frame_; }
      std::this_thread::sleep_for(50ms);
    } while (std::chrono::steady_clock::now() - t0 < timeout);
    return std::nullopt;
  }
  /// Sends a PickPlace goal and waits for it. Returns (success, message).
  std::pair<bool, std::string> runPick(const PickPlace::Goal & goal) {
    rclcpp_action::Client<PickPlace>::SendGoalOptions opts;
    opts.feedback_callback = [this](auto, PickPlace::Feedback::ConstSharedPtr f) { onStage(*f); };
    auto gh_fut = pick_place_->async_send_goal(goal, opts);
    if (gh_fut.wait_for(10s) != std::future_status::ready || !gh_fut.get()) return {false, "pick_place rejected the goal"};
    auto res_fut = pick_place_->async_get_result(gh_fut.get());
    if (res_fut.wait_for(180s) != std::future_status::ready) return {false, "pick_place timeout"};
    const auto r = res_fut.get();
    return {r.result && r.result->success, r.result ? r.result->message : "no result"};
  }
  void onStage(const PickPlace::Feedback & f) { RCLCPP_INFO(get_logger(), "   %s", f.stage.c_str()); }

  // ---- the loop ----
  void execute(std::shared_ptr<JobHandle> gh) {
    auto fb = std::make_shared<RunJob::Feedback>();
    auto res = std::make_shared<RunJob::Result>();
    const auto & req = *gh->get_goal();
    std::string state = "CAPTURE";
    int moved = 0, retries = 0;
    Dets dets;
    std::optional<PickPlace::Goal> goal;
    scene_.clear();
    while (true) {
      fb->state = state; fb->boxes_moved = moved; gh->publish_feedback(fb);
      RCLCPP_INFO(get_logger(), "[%d] %s", moved, state.c_str());
      if (gh->is_canceling()) { res->boxes_moved = moved; res->message = "cancelled"; gh->canceled(res); return; }

      if (state == "CAPTURE") {
        clearFrame();
        if (!capture()) { last_error_ = "camera timeout"; state = "RECOVER"; continue; }
        state = "DETECT";
      } else if (state == "DETECT") {
        const auto f = waitFrame(2s);
        if (!f) { last_error_ = "no detections"; state = "RECOVER"; continue; }
        dets = *f;
        if (dets.detections.empty() || (req.max_boxes >= 0 && moved >= req.max_boxes)) { state = "DONE"; continue; }
        state = "SELECT";
      } else if (state == "SELECT") {
        goal = planPick(dets);
        state = goal ? "PICK_PLACE" : "DONE";
      } else if (state == "PICK_PLACE") {
        const auto [ok, msg] = runPick(*goal);
        if (ok) { ++moved; retries = 0; recordPlace(*goal); state = "CAPTURE"; }
        else { last_error_ = msg; state = "RECOVER"; }
      } else if (state == "RECOVER") {
        ++retries;
        RCLCPP_ERROR(get_logger(), "recover (%d/3): %s", retries, last_error_.c_str());
        if (last_error_.find("still holding") != std::string::npos || retries > 3) { state = "FAILED"; continue; }
        state = "CAPTURE";
      } else {                                                          // DONE or FAILED
        res->success = state == "DONE"; res->boxes_moved = moved; res->message = state;
        if (res->success) gh->succeed(res); else gh->abort(res);
        return;
      }
    }
  }

  /// Highest box first; SKU height overrides the detector's; refuse risky grasps.
  std::optional<PickPlace::Goal> planPick(const Dets & dets) {
    auto top = [](const vision_msgs::msg::Detection3D & d) { return d.bbox.center.position.z + d.bbox.size.z / 2; };
    const auto & d = *std::max_element(dets.detections.begin(), dets.detections.end(),
                                       [&](const auto & a, const auto & b) { return top(a) < top(b); });   // highest top face
    const auto [sku, size] = matchSku(d.bbox.size.x, d.bbox.size.y);
    auto sizeOf = [](const vision_msgs::msg::Detection3D & o) { return matchSku(o.bbox.size.x, o.bbox.size.y).second; };
    Isometry3d T_box = matFromPose(d.bbox.center);
    T_box.translation().z() = top(d) - size.z / 2;
    std::vector<Neighbor> others;
    for (const auto & o : dets.detections) if (o.id != d.id) others.push_back({matFromPose(o.bbox.center), sizeOf(o)});
    const auto [risk, T_pick] = bestGrasp(T_box, size, others);
    if (risk > 0.1) { RCLCPP_WARN(get_logger(), "%s: double-pick risk %.2f, skipping", d.id.c_str(), risk); return std::nullopt; }
    const double slot = std::min_element(placed_.begin(), placed_.end(), [](const auto & a, const auto & b) { return a.second < b.second; })->first;
    if (placed_[slot] >= 2) { RCLCPP_WARN(get_logger(), "drop slots full"); return std::nullopt; }
    scene_.sync(dets, sizeOf);
    const Isometry3d T_wp = tf2::transformToEigen(buf_.lookupTransform("world", "pallet_frame", tf2::TimePointZero, tf2::durationFromSec(1.0)));
    const Isometry3d T_place = CONVEYOR_TOP * trans(0.0, slot, placed_[slot] * size.z + size.z) * rotX(M_PI);
    PickPlace::Goal g;
    g.box_id = d.id;
    g.pick_pose = stamped(T_wp * T_pick);
    g.place_pose = stamped(T_place);
    g.box_size.x = size.x; g.box_size.y = size.y; g.box_size.z = size.z;
    RCLCPP_INFO(get_logger(), "%s: sku '%s' risk %.2f -> slot y=%.1f", d.id.c_str(), sku.c_str(), risk, slot);
    return g;
  }

  /// Belief update: the box now sits in its slot; the planner must avoid it.
  void recordPlace(const PickPlace::Goal & g) {
    const double slot = std::round(g.place_pose.pose.position.y * 100) / 100;
    placed_[slot] += 1;
    const Isometry3d T = matFromPose(g.place_pose.pose) * trans(0.0, 0.0, g.box_size.z / 2);   // from TCP at the top to the center
    scene_.add("placed_" + std::to_string(slot) + "_" + std::to_string(placed_[slot]), T,
               {g.box_size.x, g.box_size.y, g.box_size.z}, "world");
  }

  rclcpp::CallbackGroup::SharedPtr cb_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_;
  rclcpp_action::Client<PickPlace>::SharedPtr pick_place_;
  rclcpp::Subscription<Dets>::SharedPtr sub_;
  rclcpp_action::Server<RunJob>::SharedPtr server_;
  tf2_ros::Buffer buf_;
  tf2_ros::TransformListener listener_;
  cell_executive::Scene scene_;
  std::mutex mtx_;
  Dets::ConstSharedPtr frame_;
  std::map<double, int> placed_;
  std::string last_error_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Executive>();
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 4);
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
}
