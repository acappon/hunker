
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "RobotNode.hpp"

std::shared_ptr<RobotNode> g_myRobotNode = nullptr;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    g_myRobotNode = std::make_shared<RobotNode>();
    rclcpp::spin(g_myRobotNode);
    rclcpp::shutdown();
    return 0;
}