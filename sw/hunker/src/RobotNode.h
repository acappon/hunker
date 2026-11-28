#ifndef ROBOT_NODE_HPP
#define ROBOT_NODE_HPP

#define JOYSTICK_DEADZONE 0.05

class RobotNode : public rclcpp::Node
{
public:
    RobotNode();
    ~RobotNode();
    void init();

    // printf-style log. An exact repeat of the previous message is not printed, only counted.
    // Each printed line starts with how many repeats of the message before it were suppressed.
    void writeLog(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
    void writeLog(const std::string &msg) { writeLog("%s", msg.c_str()); }
    bool isRobotEnabled();
    bool isImuFresh(double maxAgeSec = 0.1);   // an /imu message arrived within maxAgeSec
    void emergencyStop();

    typedef enum
    {
        LJOY_LEFT_RIGHT,
        LJOY_FWD_BACK,
        RJOY_LEFT_RIGHT,
        RJOY_FWD_BACK,
        RTRIGGER,
        LTRIGGER,
        NUMBER_OF_JOY_AXES,
    } JOY_AXES;
    double m_joy_axes[NUMBER_OF_JOY_AXES] = {};        // zero (centred) until the first /joy message

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
    bool m_joy_buttons[NUMBER_OF_JOY_BUTTONS] = {};     // current state, from the last /joy message
    bool m_joy_pressed[NUMBER_OF_JOY_BUTTONS] = {};     // latched presses, see takeButtonPress()

    // True if the button was pressed since the last call (then clears it). The 250 ms safety
    // timer uses this so a quick click between two checks isn't lost.
    bool takeButtonPress(JOY_BUTTONS button);

    // No /joy message for this long = controller lost (joy_node autorepeats at 20 Hz).
    static constexpr double kControllerTimeoutSec = 0.5;

    // No reply to the controller link probe for this long = link stale: motors are
    // disabled (not e-stopped) and B is refused until replies return. Probe reads every
    // 100 ms with ~30 ms round trip, so normal age is <= ~0.15 s. See ControllerLinkProbe.h.
    static constexpr double kLinkStaleSec = 0.4;
    bool isControllerLinkOk() const { return m_linkProbe.secondsSinceLastReply() <= kLinkStaleSec; }

    MyGpio m_myGpio;
    FaultIndicator m_faultIndicator;

    typedef enum{
        X,
        Y,
        Z,
        W,
        NUMBER_OF_AXES
    } IMU_AXES;

    std::array<double, NUMBER_OF_AXES> m_imu_orientation; 
    std::array<double, NUMBER_OF_AXES> m_imu_linear_acceleration; 
    std::array<double, NUMBER_OF_AXES> m_imu_angular_velocity; 

private: // functions
    void safetyFunction();
    void robotFunction();
 
    void checkControllerConnection();
    void checkRobotEnableDisable();
    void updateLEDs();
    void flashEnableLED();

    void enableRobot(bool isEnabled);

    std::string getStackTrace();

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    bool areAllJoysticksInDeadzone();
    void sendXboxRumble();

private: // data
    Robot m_robot;
    ControllerLinkProbe m_linkProbe;
    std::string m_lastProbeStatus;     // logged when it changes
    bool m_lastLinkOk = false;         // logged when it changes

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joystick_sub;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr m_feedback_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Time m_last_joy_msg_time;
    rclcpp::Time m_last_imu_msg_time;
    bool m_isControllerConnected;
    bool m_isRobotEnabled;
    bool m_isRobotEmergencyStopped;

    rclcpp::TimerBase::SharedPtr m_safetyTimer;
    rclcpp::TimerBase::SharedPtr m_robotTimer;

    std::string m_lastLogMsg;          // last message actually printed
    unsigned m_logRepeatCount = 0;     // exact repeats of it suppressed since
};

#endif // ROBOT_NODE_HPP