class PIDController : public rclcpp::Node
{
public:
    PIDControllerNode() : Node("pid_controller"), target_velocity_(0.0), current_velocity_(0.0);

private:
    void targetVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_velocity_ = msg->data;
    }

    void controlLoop();

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double kp_, ki_, kd_;
    double target_velocity_, current_velocity_;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
};