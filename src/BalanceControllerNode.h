
class BalanceControllerNode : public rclcpp::Node
{
public:
    BalanceControllerNode();

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void controlLoop();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double kp_, ki_, kd_;
    double target_angle_, current_angle_;
    double integral_ = 0.0;
    double previous_error_ = 0.0;
};
