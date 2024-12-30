#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "XboxControllerNode.h"

XboxControllerNode::XboxControllerNode()
    : Node("xbox_controller_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&XboxControllerNode::joy_callback, this, std::placeholders::_1));
}

void XboxControllerNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Axes: [%f, %f, %f, %f, %f, %f]",
                msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], msg->axes[4], msg->axes[5]);
    RCLCPP_INFO(this->get_logger(), "Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
                msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3],
                msg->buttons[4], msg->buttons[5], msg->buttons[6], msg->buttons[7],
                msg->buttons[8], msg->buttons[9], msg->buttons[10], msg->buttons[11]);
}
