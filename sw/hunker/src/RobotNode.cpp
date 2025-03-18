#include <chrono>
#include <dirent.h>
#include <fstream>
#include <execinfo.h>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
/*
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
*/

#include "sensor_msgs/msg/joy.hpp"

#include "MyGpio.hpp"
#include "FaultIndicator.hpp"
#include "RobotNode.hpp"


extern std::shared_ptr<RobotNode> g_myRobotNode;

RobotNode::RobotNode()
    : Node("RobotNode"), m_isControllerConnected(false), m_isRobotEnabled(false)
{
}

void RobotNode::init()
{
    try
    {
        m_isControllerConnected = false;
        m_isRobotEnabled = false;
        m_isRobotEmergencyStopped = false;

        m_myGpio.init();

        m_last_joy_msg_time = this->now() - rclcpp::Duration(10, 0);

        // Subscribe to joystick messages
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.keep_last(10)
            .reliable()
            .durability_volatile();
        m_joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", qos, std::bind(&RobotNode::joy_callback, this, std::placeholders::_1));

        m_safetyTimer = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&RobotNode::safetyFunction, this));
    }
    catch (const std::exception &e)
    {
        g_myRobotNode->writeLog("Exception in RobotNode::init()");
        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
        g_myRobotNode->writeLog(e.what());
    }
}

void RobotNode::writeLog(const std::string &msg, ...)
{
    try
    {
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
        g_myRobotNode->writeLog("Exception in RobotNode::safetyFunction");
        g_myRobotNode->writeLog(e.what());
        g_myRobotNode->writeLog("Stack trace: %s", getStackTrace().c_str());

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
                writeLog("updateLEDs: Robot ENABLED");
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

    //writeLog("controller message received, %s", std::to_string(m_last_joy_msg_time.seconds()).c_str());

    // Store the joystick data
    m_joy_axes[RJOY_FWD_BACK] = msg->axes[0];
    m_joy_axes[RJOY_LEFT_RIGHT] = msg->axes[1];
    m_joy_axes[RJOY_Z] = msg->axes[2];
    m_joy_axes[LJOY_FWD_BACK] = msg->axes[3];
    m_joy_axes[LJOY_LEFT_RIGHT] = msg->axes[4];
    m_joy_axes[LJOY_Z] = msg->axes[5];

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
