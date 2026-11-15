#!/bin/bash
cd /home/viking/hunker/sw/ros_ws

# First fix ownership of anything root may have clobbered
sudo chown -R viking:viking /home/viking/hunker/sw/ros_ws/install
sudo chown -R viking:viking /home/viking/hunker/sw/ros_ws/build

source /opt/ros/jazzy/setup.bash
colcon build --packages-select bno08x_ros2_driver
source install/setup.bash
