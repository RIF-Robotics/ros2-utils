# About

This repo is to host ROS2 nodes for additional utilities and extra functionality.

# Utilities

The following are the nodes currently available:

* **keyoard_teleop**: Publish a `Twist` command to a topic (default:
  `/cmd_vel`). Usage:

  ```
  $ source /opt/ros/foxy/setup.bash
  $ source /path/to/workspace/install/local_setup.bash
  $ ros2 run rif-ros2-utils keyboard_teleop --ros-args -r /cmd_vel:=[NEW TOPIC NAME]
  ```
