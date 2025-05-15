#ifndef ROBOT_NODE_HPP
#define ROBOT_NODE_HPP

class RobotNode : public rclcpp::Node
{
public:
    RobotNode();
    ~RobotNode();
    void init();
    void stopIMUThread();

    void writeLog(const std::string &msg, ...);
    bool isRobotEnabled();
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

    typedef enum{
        X,
        Y,
        Z,
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

private: // data
    Robot m_robot;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joystick_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Time m_last_joy_msg_time;
    rclcpp::Time m_last_imu_msg_time;
    bool m_isControllerConnected;
    bool m_isRobotEnabled;
    bool m_isRobotEmergencyStopped;

    rclcpp::TimerBase::SharedPtr m_safetyTimer;
    rclcpp::TimerBase::SharedPtr m_robotTimer;

    std::thread m_imuThread;          // Thread for IMU communication
    std::atomic<bool> m_runIMUThread; // Flag to control the thread's execution
};

#endif // ROBOT_NODE_HPP