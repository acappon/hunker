#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "bno08x_ros2_driver/bno08x_ros.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto bno08x_node = std::make_shared<BNO08x_ROS>();
        rclcpp::spin(bno08x_node);
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "Fatal: BNO08x node failed: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
