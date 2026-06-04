// Test-only common.h — shadows the production common.h so that
// production .cpp files pick up stub headers instead of real ROS2 / hardware.
#pragma once

#include <chrono>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <memory>
#include <cstdarg>

// Provide backtrace stubs (used by RobotNode::getStackTrace)
#include <execinfo.h>

// Stub ROS2 / sensor headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Production headers (order matters — mirrors the real common.h)
#include "MyGpio.h"
#include "FaultIndicator.h"
#include "BalanceDrive.h"
#include "Robot.h"
#include "RobotNode.h"
