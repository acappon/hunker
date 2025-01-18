

#ifndef ROBOTNODE_H
#define ROBOTNODE_H

class RobotNode : public rclcpp::Node
{
public:
    RobotNode();
    ~RobotNode();

    int init();
    void startRobotThread();
    void stopRobotThread();

    typedef enum
    {
        FAULT_NONE = 0,
        FAULT_ROBOT_WATCHDOG = 1,
        FAULT_NO_CONTROLLER_CONNECTED = 2,
        NUMBER_OF_FAULTS = 3
    } FAULT_TYPE;
    void setFault(RobotNode::FAULT_TYPE fault, bool isError);

    bool isControllerConnected() { return m_isControllerConnected; }
    bool isEnabled() { return m_isEnabled; }
    void setEnabled(bool enabled) { m_isEnabled = enabled; }

    void writeLogString(const std::string &msg);
    void writeLog(const char *msg);

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

public:
    static int m_lgpio_chip;

private:
    unsigned int m_faults = 0;
    bool m_fault_led_state;
    bool m_enable_led_state;
    int m_fault_index;

    bool m_isEnabled = false;
    bool m_isControllerConnected = false;

    rclcpp::TimerBase::SharedPtr m_safetyTimer;
    rclcpp::TimerBase::SharedPtr m_robotThreadTimer;

    rclcpp::Time m_last_joy_msg_time;

    void checkControllerConnection();

    void myRobotFunction();

    void safetyFunction();
    void updateEnableLED();
    void updateFaultLED();
    void setEnableLED(bool state);
    void setFaultLED(bool state);

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;
};

#endif // ROBOTNODE_H
