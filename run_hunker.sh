#!/bin/bash
# Start the whole robot: game controller, IMU and RobotNode.
# Used by hand for tests and by hunker.service at power-up.
source /opt/ros/jazzy/setup.bash
source /home/viking/hunker/sw/ros_ws/install/setup.bash
source /home/viking/hunker/sw/hunker/install/setup.bash
exec ros2 launch /home/viking/hunker/sw/hunker/launch/hunker_all.launch.py
