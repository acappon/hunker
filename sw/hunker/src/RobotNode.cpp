#include <chrono>

extern "C"
{
#include <lgpio.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "RobotNode.hpp"
#include "myGpioPins.h"

extern std::shared_ptr<RobotNode> g_myRobotNode;

int RobotNode::m_lgpio_chip = 0;

RobotNode::RobotNode()
    : Node("RobotNode"), m_isControllerConnected(false), m_isEnabled(false)
{
    // Open the GPIO chip
    m_lgpio_chip = lgGpiochipOpen(0);
    if (m_lgpio_chip < 0)
    {
        std::string msg =
            "Failed to open GPIO chip: " + std::to_string(m_lgpio_chip);
        g_myRobotNode->writeLog(msg);
    }

    if (lgGpioClaimOutput(m_lgpio_chip, 0, GPIO_PIN::GPIO_TEST, 0))
    {
        g_myRobotNode->writeLog("Failed to claim GPIO test line");
    }

    // Subscribe to joystick messages
    joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&RobotNode::joy_callback, this, std::placeholders::_1));

    this->create_wall_timer(std::chrono::seconds(1),
                            std::bind(&RobotNode::updateLEDs, this));
}

void RobotNode::writeLog(const std::string &msg)
{
    RCLCPP_INFO(this->get_logger(), msg.c_str());
}

void RobotNode::setFault(FAULT_TYPE fault, bool isFault)
{
    if (isFault)
    {
        if (std::find(m_faults.begin(), m_faults.end(), fault) ==
            m_faults.end())
        {
            m_faults.push_back(fault);
        }
    }
    else
    {
        auto it = std::find(m_faults.begin(), m_faults.end(), fault);
        if (it != m_faults.end())
        {
            m_faults.erase(it);
        }
    }
}

void RobotNode::safetyFunction()
{
    checkControllerConnection();

    updateLEDs();
}

void RobotNode::checkControllerConnection()
{
    auto now = this->now();
    bool prev = m_isControllerConnected;
    if ((now - m_last_joy_msg_time).seconds() > 2.0)
    {
        m_isControllerConnected = false;
    }
    else
    {
        m_isControllerConnected = true;
    }

    setFault(FAULT_TYPE::FAULT_CONTROLLER_DISCONNECTED, m_isControllerConnected);

    if (prev != m_isControllerConnected)
    {
        if (m_isControllerConnected)
        {
            RCLCPP_INFO(this->get_logger(), "Controller is connected");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Controller is disconnected");
        }
    }
}

void RobotNode::updateLEDs()
{
    if (!m_isControllerConnected)
    {
        setEnableLED(false);
    }
    else
    {
        if (!m_isEnabled)
        {
            setEnableLED(true);
        }
        else
        {
            flashEnableLED();
        }
    }

    static auto last_fault_time = std::chrono::steady_clock::now();
    static auto last_fault_count_time = std::chrono::steady_clock::now();

    static int prevNumberOfFaults = 0;
    static int faultIdx = 0;    // Index of the fault being displayed
    static int fault_count = 0; // Number of times the fault has been flashed
                                // for the current fault index
    auto timeDiff = std::chrono::steady_clock::now() - last_fault_time;
    static int ms_since_new_fault =
        std::chrono::duration_cast<std::chrono::milliseconds>(timeDiff).count();

    timeDiff = std::chrono::steady_clock::now() - last_fault_count_time;
    static int ms_since_fault_count_completed =
        std::chrono::duration_cast<std::chrono::milliseconds>(timeDiff).count();

    bool isFaultNew = (m_faults.size() != prevNumberOfFaults);
    prevNumberOfFaults = m_faults.size();

    if (isFaultNew)
    {
        last_fault_time = std::chrono::steady_clock::now();
        last_fault_count_time = std::chrono::steady_clock::now();
        ms_since_new_fault = 0;
        ms_since_fault_count_completed = 0;

        faultIdx = 0;
        fault_count = 0;
    }
    else
    {
        if (faultIdx >= m_faults.size())
        {
            if (ms_since_fault_count_completed > 3000)
            {
                last_fault_time = std::chrono::steady_clock::now();
                last_fault_count_time = std::chrono::steady_clock::now();
                ms_since_new_fault = 0;
                ms_since_fault_count_completed = 0;

                faultIdx = 0;
                fault_count = 0;
            }
        }
        if (fault_count >= m_faults[faultIdx])
        {
            if (ms_since_fault_count_completed > 3000)
            {
                fault_count = 0;
                faultIdx++;
            }
            else
            {
                if (ms_since_new_fault % 1000 < 500)
                {
                    setFaultLED(true);
                }
                else
                {
                    if (ms_since_new_fault % 1000 >= 500)
                    {
                        fault_count++;
                    }
                    else
                    {
                        setFaultLED(false);
                    }
                }
            }
        }
    }
}

void RobotNode::setEnableLED(bool state)
{
    if (!lgGpioWrite(g_myRobotNode->m_lgpio_chip, GPIO_PIN::GPIO_LED_ENABLE,
                     state ? 1 : 0))
    {
        g_myRobotNode->writeLog("Failed to write to LED enable GPIO");
    }
}

void RobotNode::flashEnableLED()
{
    int millisecs = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count() %
                    1000;
    setEnableLED(millisecs < 500);
}

void RobotNode::setFaultLED(bool state)
{
    if (!lgGpioWrite(g_myRobotNode->m_lgpio_chip, GPIO_PIN::GPIO_LED_FAULT,
                     state ? 1 : 0))
    {
        g_myRobotNode->writeLog("Failed to write to LED enable GPIO");
    }
}

void RobotNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Listen for Joystick messages
    RCLCPP_INFO(this->get_logger(), "Axes: [%f, %f, %f, %f, %f, %f]",
                msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3],
                msg->axes[4], msg->axes[5]);
    RCLCPP_INFO(this->get_logger(),
                "Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
                msg->buttons[0], msg->buttons[1], msg->buttons[2],
                msg->buttons[3], msg->buttons[4], msg->buttons[5],
                msg->buttons[6], msg->buttons[7], msg->buttons[8],
                msg->buttons[9], msg->buttons[10], msg->buttons[11]);

    m_isControllerConnected = true;
    m_last_joy_msg_time = this->now();

    // Store the joystick data
    m_joy_axes[RJOY_FWD_BACK] = msg->axes[0];
    m_joy_axes[RJOY_LEFT_RIGHT] = msg->axes[1];
    m_joy_axes[RJOY_Z] = msg->axes[2];
    m_joy_axes[LJOY_FWD_BACK] = msg->axes[3];
    m_joy_axes[LJOY_LEFT_RIGHT] = msg->axes[4];
    m_joy_axes[LJOY_Z] = msg->axes[5];

    m_joy_buttons[JOY_A] = msg->buttons[0];
    m_joy_buttons[JOY_B] = msg->buttons[1];
    m_joy_buttons[JOY_X] = msg->buttons[2];
    m_joy_buttons[JOY_Y] = msg->buttons[3];
    m_joy_buttons[JOY_LB] = msg->buttons[4];
    m_joy_buttons[JOY_RB] = msg->buttons[5];
    m_joy_buttons[JOY_BACK] = msg->buttons[6];
    m_joy_buttons[JOY_START] = msg->buttons[7];
    m_joy_buttons[JOY_LOGITECH] = msg->buttons[8];
    m_joy_buttons[JOY_L3] = msg->buttons[9];
    m_joy_buttons[JOY_R3] = msg->buttons[10];
}
