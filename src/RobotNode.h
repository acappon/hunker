

class RobotNode : public rclcpp::Node
{
public:
    RobotNode();
    ~RobotNode();

    void init();
    void writeLog(const std::string& msg);
    void writeLog(const char* msg);

    void periodic();
    void disabledPeriodic();
    void balancingPeriodic();
    void airbornePeriodic();
    void landingPeriodic();
 
public:
    IMU_bno055 imu;

    Motor* pMotors;
 
private:
    bool m_isEnabled = false;
    double m_deck_pitch;
    double m_deck_roll;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    // radians
    // knee motor turn counts indicate leg extension.
    // The deck would be in this orientation if balanced vertically
    void estimateDeckOrientation();
 
    bool isBalancingPossible(); // true if the robot is not airborne, and the deck vertical is not too far from "up"
    bool isFullyFolded(); // true if the robot is in a compact, folded state, (lower limit switch is active)
};
