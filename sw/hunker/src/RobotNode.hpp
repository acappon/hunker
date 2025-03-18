#ifndef ROBOT_NODE_HPP
#define ROBOT_NODE_HPP

class RobotNode : public rclcpp::Node
{
public:
    RobotNode();
    void init();

    void writeLog(const std::string &msg, ...);
    bool isRobotEnabled();
    void emergencyStop();

    typedef enum
    {
        RJOY_FWD_BACK,
        RJOY_LEFT_RIGHT,
        RJOY_Z,
        LJOY_FWD_BACK,
        LJOY_LEFT_RIGHT,
        LJOY_Z,
        NUMBER_OF_JOY_AXES,
    } JOY_AXES;
    double m_joy_axes[NUMBER_OF_JOY_AXES];

    typedef enum
    {
        JOY_A,
        JOY_B,
        JOY_X,
        JOY_Y,
        JOY_LB,
        JOY_RB,
        JOY_BACK,
        JOY_START,
        JOY_LOGITECH,
        JOY_L3,
        JOY_R3,
        NUMBER_OF_JOY_BUTTONS,
    } JOY_BUTTONS;
    bool m_joy_buttons[12];

    MyGpio m_myGpio;
    FaultIndicator m_faultIndicator;

private: // functions
    void safetyFunction();
    void checkControllerConnection();
    void checkRobotEnableDisable();
    void updateLEDs();
    void flashEnableLED();
 
    void enableRobot(bool isEnabled);

    std::string getStackTrace();

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

private: // data
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joystick_sub;

    rclcpp::Time m_last_joy_msg_time;
    bool m_isControllerConnected;
    bool m_isRobotEnabled;
    bool m_isRobotEmergencyStopped;

    rclcpp::TimerBase::SharedPtr m_safetyTimer;
};

#endif // ROBOT_NODE_HPP