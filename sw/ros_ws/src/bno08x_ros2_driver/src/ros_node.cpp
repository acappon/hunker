#include <rclcpp/rclcpp.hpp>
#include "bno08x_ros2_driver/bno08x_ros.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<BNO08x_ROS> bno08x_node = std::make_shared<BNO08x_ROS>();
    rclcpp::spin(bno08x_node);
    rclcpp::shutdown();

    return 0;
}
