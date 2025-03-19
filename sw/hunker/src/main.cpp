#include "rclcpp/rclcpp.hpp"
#include <csignal>
#include "sensor_msgs/msg/joy.hpp"
#include "MyGpio.hpp"
#include "FaultIndicator.hpp"
#include "Motor.h"
#include "IMU_bno055.h"
#include "Robot.h"
#include "RobotNode.hpp"

std::shared_ptr<RobotNode> g_myRobotNode = nullptr;

void signal_handler(int signum)
{
    try
    {
        g_myRobotNode->writeLog("Signal %d received, shutting down...", signum);
        g_myRobotNode->emergencyStop();
    }
    catch (const std::exception &e)
    {
        // ignore exception
    }
    exit(1);
}

int main(int argc, char *argv[])
{
    // Register signal(s) handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
 
    rclcpp::init(argc, argv);
    g_myRobotNode = std::make_shared<RobotNode>();
    g_myRobotNode->init();

    rclcpp::spin(g_myRobotNode);
    rclcpp::shutdown();
    return 0;
}