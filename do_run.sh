#!/bin/bash

source /home/viking/sw/hunker/install/setup.bash

ros2 run joy joy_node
ros2 run hunker hunker_node