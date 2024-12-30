#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "BalanceControllerNode.h"
#include <Quaternion.h>

BalanceControllerNode::BalanceControllerNode()
    : Node("balance_controller"), target_angle_(0.0), current_angle_(0.0)
{
    this->declare_parameter<double>("kp", 1.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, std::bind(&BalanceControllerNode::imuCallback, this, std::placeholders::_1));

    left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_motor/command", 10);
    right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_motor/command", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&BalanceControllerNode::controlLoop, this));
}

void BalanceControllerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Assuming the IMU orientation is in quaternion format and we are interested in the pitch angle
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_angle_ = pitch;
}

void BalanceControllerNode::controlLoop()
{
    double error = target_angle_ - current_angle_;
    double p_term = kp_ * error;
    integral_ += error * 0.01; // Assuming control loop runs every 10ms
    double i_term = ki_ * integral_;
    double d_term = kd_ * (error - previous_error_) / 0.01;

    double motor_command = p_term + i_term + d_term;

    auto left_command_msg = std_msgs::msg::Float64();
    auto right_command_msg = std_msgs::msg::Float64();
    left_command_msg.data = motor_command;
    right_command_msg.data = motor_command;

    left_motor_pub_->publish(left_command_msg);
    right_motor_pub_->publish(right_command_msg);

    previous_error_ = error;
}
