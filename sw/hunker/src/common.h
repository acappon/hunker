#include <chrono>
#include <dirent.h>
#include <fstream>
#include <execinfo.h>
#include <cstring>
#include <termios.h> // For UART configuration

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "MyGpio.h"
#include "FaultIndicator.h"
#include "Motor.h"
#include "Robot.h"
#include "RobotNode.h"