
#include "rclcpp/rclcpp.hpp"
#include "PIDControllerNode.h"
#include <float64__struct.hpp>

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode() : Node("pid_controller"), target_velocity_(0.0), current_velocity_(0.0)
    {
        this->declare_parameter<double>("kp", 1.0);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.0);

        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();

        target_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "target_velocity", 10, std::bind(&PIDController::targetVelocityCallback, this, std::placeholders::_1));

        motor_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("motor_command", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PIDController::controlLoop, this));
    }

private:
    void targetVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_velocity_ = msg->data;
    }

    void controlLoop()
    {
        double error = target_velocity_ - current_velocity_;
        double p_term = kp_ * error;
        integral_ += error * 0.1; // Assuming control loop runs every 100ms
        double i_term = ki_ * integral_;
        double d_term = kd_ * (error - previous_error_) / 0.1;

        double motor_command = p_term + i_term + d_term;

        auto command_msg = std_msgs::msg::Float64();
        command_msg.data = motor_command;
        motor_command_pub_->publish(command_msg);

        previous_error_ = error;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double kp_, ki_, kd_;
    double target_velocity_, current_velocity_;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
};