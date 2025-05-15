#include "common.h"

extern std::shared_ptr<RobotNode> g_myRobotNode;

RobotNode::RobotNode()
    : Node("RobotNode"), m_isControllerConnected(false), m_isRobotEnabled(false)
{
}

RobotNode::~RobotNode()
{
    stopIMUThread();
}

void RobotNode::init()
{
    std::string sRet;

    try
    {
        m_isControllerConnected = false;
        m_isRobotEnabled = false;
        m_isRobotEmergencyStopped = false;

        m_myGpio.initEnableAndFaultLED();
        m_robot.init();

        m_last_joy_msg_time = m_last_imu_msg_time = this->now() - rclcpp::Duration(10, 0);

        // Subscribe to joystick messages
        auto qos_joy = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_joy.keep_last(10)
            .best_effort()
            .durability_volatile();
        m_joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", qos_joy, 
            std::bind(&RobotNode::joy_callback, this, std::placeholders::_1));

        // Create a subscription to the /imu topic
        auto qos_imu = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_imu.keep_last(10)
            .reliable()
            .durability_volatile();
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos_imu,
            std::bind(&RobotNode::imu_callback, this, std::placeholders::_1));

        m_safetyTimer = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&RobotNode::safetyFunction, this));

        m_robotTimer = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RobotNode::robotFunction, this));
    }
    catch (const std::exception &e)
    {
        writeLog("Exception in RobotNode::init()");
        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
        writeLog(e.what());
    }
}

void RobotNode::stopIMUThread()
{
    m_runIMUThread = false; // Signal the thread to stop
    if (m_imuThread.joinable())
    {
        m_imuThread.join(); // Wait for the thread to finish
    }
}

void RobotNode::writeLog(const std::string &msg, ...)
{
    try
    {
        static std::string prevMsg;
        if(msg == prevMsg)
        {
            return; // Avoid duplicate messages
        }   
        prevMsg = msg;
        if (msg.empty())
        {
            return; // Avoid empty messages
        }
        if (msg.length() > 1000)
        {
            writeLog("Message too long, truncating to 1000 characters");
            return;
        }
        
        va_list vl;
        va_start(vl, msg);
        RCLCPP_INFO(this->get_logger(), msg.c_str(), vl);
        va_end(vl);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception in RobotNode::writeLog: %s", e.what());
        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
    }
}

bool RobotNode::isRobotEnabled()
{
    return m_isRobotEnabled;
}

void RobotNode::safetyFunction()
{
    try
    {
        if (m_isRobotEmergencyStopped)
        {
            emergencyStop();
        }
        else
        {
            checkControllerConnection();
        }

        if (isRobotEnabled() && (!m_isControllerConnected))
        {
            m_isRobotEmergencyStopped = true;
        }

        updateLEDs();

        checkRobotEnableDisable();
    }
    catch (const std::exception &e)
    {
        writeLog("Exception in RobotNode::safetyFunction");
        writeLog(e.what());
        writeLog("Stack trace: %s", getStackTrace().c_str());

        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
    }
}

void RobotNode::robotFunction()
{
    try
    {
        m_robot.robotFunction();
    }
    catch (const std::exception &e)
    {
        writeLog("Exception in Robot::robotFunction");
        writeLog(e.what());
        writeLog("Stack trace: %s", getStackTrace().c_str());

        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
    }
}

void RobotNode::emergencyStop()
{
    m_isRobotEmergencyStopped = true;
    m_isRobotEnabled = false;
    m_myGpio.enableWheelMotors(false);
}

void RobotNode::enableRobot(bool isEnabled)
{
    if (m_isRobotEmergencyStopped)
    {
        // Cycle power to clear emergency stop
        return;
    }
    m_isRobotEnabled = isEnabled;
    m_myGpio.enableWheelMotors(isEnabled);
}

void RobotNode::checkControllerConnection()
{
    rclcpp::Time now = this->now();
    bool prev = m_isControllerConnected;
    if (abs((now - m_last_joy_msg_time).seconds()) > 2.0)
    {
        writeLog("Controller is disconnected, now = %d, last = %d",
                 now.nanoseconds(),
                 m_last_joy_msg_time.nanoseconds());
        m_isControllerConnected = false;
    }
    else
    {
        m_isControllerConnected = true;
    }

    m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_CONTROLLER_DISCONNECTED, !m_isControllerConnected);

    if (prev != m_isControllerConnected)
    {
        if (m_isControllerConnected)
        {
            writeLog("Controller is connected");
        }
        else
        {
            writeLog("Controller is disconnected");
        }
    }
}

void RobotNode::checkRobotEnableDisable()
{
    if (m_joy_buttons[JOY_B] == 1)
    {
        if (!isRobotEnabled())
        {
            writeLog("Robot ENABLED");
            enableRobot(true);
        }
    }

    if (m_joy_buttons[JOY_X] == 1)
    {
        if (isRobotEnabled())
        {
            writeLog("Robot DISABLED");
            enableRobot(false);
        }
    }

    if ((m_joy_buttons[JOY_LB] == 1) || (m_joy_buttons[JOY_RB] == 1))
    {
        emergencyStop();
    }
}

void RobotNode::updateLEDs()
{
    if (m_isRobotEmergencyStopped)
    {
        writeLog("updateLEDs: ESTOP");

        m_myGpio.setEnableLED(false);
        m_myGpio.setFaultLED(true);
    }
    else
    {
        if (!m_isControllerConnected)
        {
            writeLog("updateLEDs: XBox NOT connected");
            m_myGpio.setEnableLED(false);
        }
        else
        {
            if (!m_isRobotEnabled)
            {
                writeLog("updateLEDs: Robot disabled");
                m_myGpio.setEnableLED(true);
            }
            else
            {
                writeLog("updateLEDs: Robot ENABLED, state %s", m_robot.getRobotStateText().c_str());
                flashEnableLED();
            }
        }
        m_faultIndicator.update();
    }
}

void RobotNode::flashEnableLED()
{
    int millisecs = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count() %
                    1000;
    m_myGpio.setEnableLED(millisecs < 500);
}

std::string RobotNode::getStackTrace()
{
    try
    {
        {
            void *array[100];
            int size;

            // Get the backtrace
            size = backtrace(array, 10);
            return *backtrace_symbols(array, size);
        }
    }
    catch (const std::exception &e)
    {
        // ignore exception
    }
    return "EXCEPTION in getStatckTrace()";
}

void RobotNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Listen for Joystick messages
    /*
    writeLog("Axes: [%f, %f, %f, %f, %f, %f]",
             msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3],
             msg->axes[4], msg->axes[5]);
    writeLog("Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
             msg->buttons[0], msg->buttons[1], msg->buttons[2],
             msg->buttons[3], msg->buttons[4], msg->buttons[5],
             msg->buttons[6], msg->buttons[7], msg->buttons[8],
             msg->buttons[9], msg->buttons[10], msg->buttons[11]);
    */

    m_isControllerConnected = true;

    m_last_joy_msg_time = this->now();

    // writeLog("controller message received, %s", std::to_string(m_last_joy_msg_time.seconds()).c_str());

    // Store the joystick data
    m_joy_axes[LJOY_LEFT_RIGHT] = msg->axes[0];
    m_joy_axes[LJOY_FWD_BACK] = msg->axes[1];
    m_joy_axes[RJOY_LEFT_RIGHT] = msg->axes[2];
    m_joy_axes[RJOY_FWD_BACK] = msg->axes[3];
    m_joy_axes[RTRIGGER] = msg->axes[4];
    m_joy_axes[LTRIGGER] = msg->axes[5];

    m_joy_buttons[JOY_A] = msg->buttons[0];
    m_joy_buttons[JOY_B] = msg->buttons[1];
    // m_joy_buttons[JOY_UNUSED_1] = msg->buttons[2];
    m_joy_buttons[JOY_X] = msg->buttons[3];
    m_joy_buttons[JOY_Y] = msg->buttons[4];
    // m_joy_buttons[JOY_UNUSED_2] = msg->buttons[5];
    m_joy_buttons[JOY_LB] = msg->buttons[6];
    m_joy_buttons[JOY_RB] = msg->buttons[7];

    m_joy_buttons[JOY_BACK] = msg->buttons[8];
    m_joy_buttons[JOY_START] = msg->buttons[9];
    m_joy_buttons[JOY_LOGITECH] = msg->buttons[10];
    m_joy_buttons[JOY_L3] = msg->buttons[11];
    m_joy_buttons[JOY_R3] = msg->buttons[12];
}

// Callback function for IMU messages
void RobotNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_last_imu_msg_time = this->now();

    m_imu_orientation[IMU_AXES::X] = msg->orientation.x;
    m_imu_orientation[IMU_AXES::Y] = msg->orientation.y;
    m_imu_orientation[IMU_AXES::Z] = msg->orientation.z;

    m_imu_linear_acceleration[IMU_AXES::X] = msg->linear_acceleration.x;
    m_imu_linear_acceleration[IMU_AXES::Y] = msg->linear_acceleration.y;
    m_imu_linear_acceleration[IMU_AXES::Z] = msg->linear_acceleration.z;

    m_imu_angular_velocity[IMU_AXES::X] = msg->angular_velocity.x;
    m_imu_angular_velocity[IMU_AXES::Y] = msg->angular_velocity.y;
    m_imu_angular_velocity[IMU_AXES::Z] = msg->angular_velocity.z;
}
