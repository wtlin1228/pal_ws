# Appendix — cheat sheet, troubleshooting, answers, sources

## Commands you will type daily

```bash
cd ~/pal_ws && colcon build --symlink-install && source install/setup.bash
colcon build --packages-select cell_planning          # one package
colcon test --packages-select cell_planning cell_perception && colcon test-result --verbose   # GoogleTest
ros2 launch cell_bringup cell.launch.py                # the whole cell
ros2 run cell_sim spawn_scene --scene single_sku --layers 2
ros2 action send_goal /run_job cell_interfaces/action/RunJob "{mode: depalletize, strategy: single_sku}" --feedback
ros2 topic echo /detections --once | head -40
ros2 run tf2_ros tf2_echo world tcp
ros2 run tf2_tools view_frames
ros2 control list_controllers
ros2 param set /mock_gripper fail_every_n 3
rqt_graph; rqt_console; ros2 doctor --report
```

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| RViz black or crashes in the VM | no GPU OpenGL | `export LIBGL_ALWAYS_SOFTWARE=1` |
| `ros2 topic echo /camera/points` shows nothing | QoS mismatch | subscribe best-effort (`--qos-reliability best_effort`, RViz Reliability Policy) |
| `Lookup would require extrapolation` | asked for a stamp the buffer lacks | use `tf2::TimePointZero` for latest or a timeout; check `/tf_static` is published |
| Cloud floats above the robot or is mirrored | optical frame rotation wrong | `rpy="${pi} 0 0"` on `camera_optical_frame`; z must point down in RViz |
| `controller_manager` has no interfaces | `ros2_control_node` never got the URDF | the `~/robot_description` remapping, or two `<ros2_control>` blocks |
| MoveIt: goal state in collision | pad touching the target box | remove the target object before the descend (Ch 11) |
| Cartesian path fraction < 1 | collision or IK discontinuity on the line | raise the approach, `use_pilz = true`, check joint limits |
| `No IK solution` near the pallet's far corner | reach | move the pallet 5 cm closer or lower the pedestal |
| A future never completes inside a callback | single-threaded executor | `MultiThreadedExecutor` + a `Reentrant` callback group, or a thread (Ch 1) |
| Two boxes detected as one | gap narrower than the point spacing | widen the gap in the scene; on a real camera the edges split them |
| `spawn_scene` boxes appear in the wrong place | spawned before TF was available | start the twin after `robot_state_publisher` |
| Attached box stays after a crash | stale scene | detach all attached objects at startup (Ch 16) |
| apt: Unable to locate package ros-jazzy-… | not built for your architecture or distro yet | drop that package from the command and rerun; of the UR packages only `ur_description` is required |
| colcon build: `move_group_interface.hpp: No such file` | MoveIt older than 2.10 ships `.h` headers; or the planning-interface package is missing | use the `.h` name, or `sudo apt install ros-jazzy-moveit-ros-planning-interface` |
| `MoveGroupInterface`: *Robot model not loaded* | the node has no robot description | start the node with `moveit_config.to_dict()` parameters, or check `move_group` publishes `/robot_description_semantic` |

## Answers to the chapter questions

Try each question before reading its answer; the point of a checkpoint is to find out what you can explain without looking.

**Ch 0.** (1) The domain ID partitions the DDS network: nodes only discover others in the same domain. (2) ROS 2 discovery is multicast on the LAN, so your colleague's `talker` would join your graph and your `listener` would print their messages.

**Ch 1.** (1) A service: it is short and has no meaningful intermediate state. (2) `declare_parameter` only stores the value; nothing re-reads it unless you register a parameter callback.

**Ch 2.** (1) The executive (Ch 13), using the grasp rules of Ch 11. (2) The grasp strategy belongs to the application; a motion server that only knows TCP poses works unchanged for any gripper or strategy.

**Ch 3.** (1) Box heights become "z above the deck": layer 1 starts at z = 0. (2) Every planner would see an extra degree of freedom with no actuator behind it.

**Ch 4.** (1) The world origin as seen from the TCP: the inverse of the usual lookup. (2) `/tf_static` is sent once with transient-local QoS; a dynamic transform expires after `cache_time` (10 s by default), so a moving box must be re-published continuously.

**Ch 5.** (1) The trajectory controller uses velocity for its stopped-velocity check; the mock reports zero velocity, which passes. (2) Only the hardware plugin: `gz_ros2_control/GazeboSimSystem` (Ch 17).

**Ch 6.** (1) So that pose goals are pad poses; otherwise you offset every goal by 0.12 m yourself. (2) Pairs that are adjacent or can never touch are skipped at planning time; a wrong entry lets the planner drive one link into another.

**Ch 7.** (1) The straight line crosses a collision or an IK discontinuity: check the logged fraction, then try `use_pilz = true` or a higher approach. (2) The program must block on each motion while callbacks keep running in the executor thread.

**Ch 8.** (1) One internal frame keeps the mock physics simple; callers still think in `pallet_frame`. (2) Corners would be missed or false hits occur; the same bug would appear in the Ch 15 footprint checks.

**Ch 9.** (1) The camera publishes with sensor-data (best-effort) QoS; a reliable subscriber is incompatible and silently receives nothing. (2) The footprint corners: an axis-aligned box misjudges a rotated one.

**Ch 10.** (1) The pick order is top-down anyway, and lower boxes are partly occluded, so their rectangles would be wrong. (2) An L shape: two boxes merged at a corner, or a box plus a leaked lower neighbor.

**Ch 11.** (1) A 0.4 m box on the pad changes the swept volume; without it the planner clips the conveyor edge or the stack. (2) A stale attached object blocks every plan; Ch 16 clears the scene at startup.

**Ch 12.** (1) A box placed near the robot would sit under the arm's path to the far row, and the planner would have to go around it. (2) Nothing: `bestGrasp` already yaws the TCP with the box, and the place pose inherits the pattern's yaw, so the wrist turns 90° in transit.

**Ch 13.** (1) The top view cannot see the bottom of a box; the WMS can. (2) A reentrant callback group with a multithreaded executor, and the job runs in its own thread that waits on futures instead of spinning.

**Ch 14.** (1) A command with a completion is request/reply; a sensor state is a stream. (2) Pad overhang over the belt edge is harmless, but an off-center grasp shifts the box within its pattern slot and the stack drifts layer by layer.

**Ch 15.** (1) Touching a neighbor must block the placement, but a halo is not a surface a box can stand on. (2) It only costs a few milliseconds; removing it is an optimization, not a correctness fix.

**Ch 16.** (1) The executive would stop and wait for a person although the gripper is empty and work can continue. (2) Timing and closed-loop behavior: the recorded cloud never changes in response to your picks.

**Ch 17.** Truth lived in one node behind generic interfaces and the robot software never read it, so only `cell_sim` changes.

## Verified against

The pure algorithms of this tutorial (geometry, grasps, mock camera, detector, patterns, mixed placement) were run against the scenes described in Ch 8 before publication, in their Python form; the C++ versions are line-by-line ports of the same logic and could not be compiled against ROS 2 here, so expect the odd include path or signature to need a touch-up on first build. ROS 2 facts come from these pages, read on 2026-11-08:

- [ROS 2 Lyrical Luth released (May 2026, LTS to 2031, Ubuntu 26.04)](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021)
- [Lyrical Luth release page: platforms and Gazebo Jetty](https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html)
- [ur_robot_driver usage: mock hardware and `initial_joint_controller:=joint_trajectory_controller`](https://docs.ros.org/en/jazzy/p/ur_robot_driver/doc/usage/move.html)
- [ur_robot_driver startup arguments](https://docs.ros.org/en/jazzy/p/ur_robot_driver/doc/usage/startup.html)
- [`ur_macro.xacro` (Jazzy): `ur_robot` macro parameters](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/jazzy/urdf/ur_macro.xacro)
- [`sensor_msgs_py.point_cloud2` (Jazzy): `read_points_numpy`, `create_cloud_xyz32`](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs_py/sensor_msgs_py/point_cloud2.py)

Not verified here: the exact MoveIt Setup Assistant page names (stable across Humble–Jazzy), apt package availability for arm64 on every package listed in Ch 0 (`ros-jazzy-ur` metapackage in particular; `ur_description` alone suffices), and Gazebo details in Ch 17.
