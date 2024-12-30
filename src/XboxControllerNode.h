class XboxControllerNode : public rclcpp::Node
{
public:
    XboxControllerNode();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};
