

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "XboxControllerNode.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XboxControllerNode>());
    rclcpp::shutdown();
    return 0;
}