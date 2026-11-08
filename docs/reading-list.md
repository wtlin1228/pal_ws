# Reading list — what existing docs, books and papers add to this tutorial

Surveyed on 2026-11-08: the official ROS 2 Jazzy tutorials, MoveIt 2 tutorials, ros2_control docs, the Universal Robots ROS 2 tutorials, two current ROS 2 books, Zivid's (de)palletization guide, and the recent research on robot bin packing and parcel detection. Verdict: the chapter structure holds; seven things were worth borrowing, and they are linked from the chapters they belong to ("Go deeper" sections).

## Worth borrowing

| Resource | What it adds | Lands in |
| --- | --- | --- |
| [UR ROS 2 tutorials: Custom workcell](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_tutorials/my_robot_cell/doc/index.html) | The vendor's own version of Ch 3–6: a workcell description package, the driver started with that description (mock or real), and a MoveIt config on top. Same three-package split as ours, so it is the shortest path from this tutorial to a real UR. | Ch 3, 5, 6, 18 |
| [MoveIt 2 examples index](https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/examples.rst): Pick and Place with MoveIt Task Constructor, Planning Scene ROS API, Perception Pipeline, Hand-Eye Calibration | MTC is MoveIt's framework for exactly the stage sequence `pick_place_server` hand-rolls; the perception pipeline builds an octomap from the camera so the planner avoids the whole stack; hand-eye calibration is the step every real camera needs. | Ch 11, 13, 18 |
| [ros2_control: Writing a hardware component](https://docs.ros.org/en/jazzy/p/hardware_interface/doc/writing_new_hardware_component.html) and [demos example 7](https://control.ros.org/jazzy/doc/api/classros2__control__demo__example__7_1_1RobotSystem.html) (a 6-axis arm with its own hardware plugin and controller) | What replaces `mock_components/GenericSystem` when you talk to a controller yourself: `on_init`, `read()`, `write()`. | Ch 5, 18 |
| [Zivid: Depalletization and Palletization Tutorial](https://support.zivid.com/en/latest/camera/academy/applications/depalletization-tutorial.html) | A camera vendor's field guide: which object features a camera must see, camera placement for stationary vs on-arm mounting, 2D + 3D capture strategy, settings against a time budget, capturing while the robot moves to cut cycle time, production preparation (warm-up, in-field verification, hand-eye, ROI filtering) and maintenance. | Ch 9, 13, 18 |
| [RoboBPP benchmark](https://arxiv.org/pdf/2512.04415) (arXiv 2512.04415, 2026) | An open benchmark for robotic online bin packing with a physics simulator, three real industrial box datasets and a leaderboard: realistic box sequences for Ch 15 and a physics check for Ch 17. | Ch 15, 17 |
| [Wang & Hauser, ICRA 2019](https://motion.cs.illinois.edu/papers/ICRA2019-Wang-BinPacking.pdf) and [Gao et al., 2025](https://web3.arxiv.org/pdf/2507.09123) | The heightmap-minimization heuristic that Ch 15 simplifies, with stability and robot-packability constraints; a fast stability validation for online packing. | Ch 15 |
| [Monica, Aleotti, Lodi Rizzini, IRC 2020](https://www.ce.unipr.it/~rizzini/papers/monica20irc.pdf) | Box detection for tightly packed parcels from a ToF camera using edges from the intensity image plus depth discontinuities: what to do when the gaps Ch 10 relies on are not there. | Ch 10 |

## General companions

| Resource | Use it for |
| --- | --- |
| [ROS 2 Jazzy tutorials, Intermediate](https://docs.ros.org/en/jazzy/Tutorials/Intermediate.html) and [launch for large projects](https://docs.ros.org/en/jazzy/Tutorials/Launch/Using-ROS2-Launch-for-Large-Projects.html) | The official versions of Ch 1, 2, 4 and 16: tf2 broadcasters and listeners, launch substitutions and event handlers, `launch_testing`, recording and reading bags from a node. |
| [Mastering ROS 2 for Robotics Programming, 4th ed. (Packt, July 2025)](https://www.packtpub.com/product/mastering-ros-2-for-robotics-programming/9781836209010) | Jazzy-based book with chapters on ros2_control, BehaviorTree.CPP, MoveIt 2, the perception stack and CI; the natural next book after this tutorial. |
| [A Concise Introduction to Robot Programming with ROS 2 (CRC Press, 2025)](https://service-library.mfu.ac.th/bookshop/detail.php?id=4823) | C++-first fundamentals, a complete simulated robot, testing strategies and behavior trees; the same language choice as this tutorial. |
| [Mike Likes Robots: MoveIt Task Constructor walkthrough](https://mikelikesrobots.github.io/blog/moveit-task-constructor) and [Automatic Addison: pick and place with MTC and perception (Jazzy)](https://automaticaddison.com/?p=17329) | Two readable, working MTC examples, one with PCL-based perception, if you take the Ch 11 optional port. |
| [Zivid: robotic (de)palletization overview](https://www.zivid.com/applications/robotic-palletization-and-depalletization-with-3d-vision) | What production cells look like, including a Mujin depalletizing demo with a Zivid camera. |

## Not borrowed

MoveIt Pro, Isaac Sim and learned (CNN) box segmentation were left out: they are either commercial, GPU-bound in a VM, or need training data you do not have. The structure here lets you add any of them later behind the same `/detections` and `PickPlace` interfaces.
