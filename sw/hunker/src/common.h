#include <chrono>
#include <dirent.h>
#include <fstream>
#include <execinfo.h>
#include <cstring>
#include <termios.h> // For UART configuration

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"

#include "MyGpio.h"
#include "FaultIndicator.h"
#include "Motor.h"
#include "BNO080.h"
#include "Robot.h"
#include "RobotNode.h"