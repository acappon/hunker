
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "MyGpio.hpp"
#include "FaultIndicator.hpp"
#include "RobotNode.hpp"

std::shared_ptr<RobotNode> g_myRobotNode = nullptr;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    g_myRobotNode = std::make_shared<RobotNode>();
    g_myRobotNode->init();
    rclcpp::spin(g_myRobotNode);
    rclcpp::shutdown();
    return 0;
}