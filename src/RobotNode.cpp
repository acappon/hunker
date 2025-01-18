#include <iostream>
#include <fstream>
#include <csignal>
#include <memory>

extern "C"
{
#include <lgpio.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "IMU_bno055.h"
#include "myGpioPins.h"
#include "Motor.h"
#include "Robot.h"
#include "RobotNode.h"

std::shared_ptr<RobotNode> g_myRobotNode = nullptr;
std::shared_ptr<Robot> g_myRobot = nullptr;
int RobotNode::m_lgpio_chip = 0;

RobotNode::RobotNode()
    : Node("robot_node")
{
    init();
}

int RobotNode::init()
{
    // Open the GPIO chip
    m_lgpio_chip = lgGpiochipOpen(0);
    if (m_lgpio_chip < 0)
    {
        std::string msg = "Failed to open GPIO chip: " + std::to_string(m_lgpio_chip);
        g_myRobotNode->writeLogString(msg);
        return 1;
    }

    if (lgGpioClaimOutput(m_lgpio_chip, 0, GPIO_PIN::GPIO_TEST, 0))
    {
        g_myRobotNode->writeLog("Failed to claim GPIO test line");
        return 1;
    }

    // Subscribe to joystick messages
    subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&RobotNode::joy_callback, this, std::placeholders::_1));

    m_safetyTimer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RobotNode::safetyFunction, this));

    startRobotThread();

    return 0;
}

void RobotNode::myRobotFunction()
{
    // This is wierd, I know, but the timer function won't compile if you provide a 
    // callback function outsiude the class it is called from
    g_myRobot->robotFunction();
}


void RobotNode::safetyFunction()
{
    checkControllerConnection();

    // Update LEDs based on faults and enable status
    updateEnableLED();
    updateFaultLED();
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

    g_myRobotNode->setFault(RobotNode::FAULT_NO_CONTROLLER_CONNECTED, m_isControllerConnected);

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

void RobotNode::updateEnableLED()
{
    if (!m_isEnabled)
    {
        m_enable_led_state = true; // Solid on
    }
    else
    {
        m_enable_led_state = !m_enable_led_state; // Flashing
    }

    setEnableLED(m_enable_led_state);
}

void RobotNode::updateFaultLED()
{
    if (m_faults == 0)
    {
        m_fault_led_state = false; // No faults, LED off
        m_fault_index = 0;
    }
    else
    {
        if (m_fault_index < NUMBER_OF_FAULTS && (m_faults & (1 << m_fault_index)) != 0)
        {
            m_fault_led_state = !m_fault_led_state; // Flashing to count out fault number
        }
        else
        {
            m_fault_led_state = false; // Pause for 2 seconds
            m_fault_index++;
            if (m_fault_index >= NUMBER_OF_FAULTS)
            {
                m_fault_index = 0;
            }
        }
    }

    // Set the fault LED state (replace with actual hardware control)
    setFaultLED(m_fault_led_state);
}

void RobotNode::setEnableLED(bool state)
{
    if(!lgGpioWrite(g_myRobotNode->m_lgpio_chip, GPIO_PIN::GPIO_LED_ENABLE, state ? 1 : 0))
    {
        g_myRobotNode->writeLog("Failed to write to LED enable GPIO");
    }
}

void RobotNode::setFaultLED(bool state)
{
    if(!lgGpioWrite(g_myRobotNode->m_lgpio_chip, GPIO_PIN::GPIO_LED_FAULT, state ? 1 : 0))
    {
        g_myRobotNode->writeLog("Failed to write to LED enable GPIO");
    }
}

void RobotNode::startRobotThread()
{
     m_robotThreadTimer = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&RobotNode::myRobotFunction, this));
}

void RobotNode::stopRobotThread()
{
    m_robotThreadTimer.reset();
}

void RobotNode::setFault(RobotNode::FAULT_TYPE fault, bool isError)
{
    if (isError)
    {
        m_faults |= fault;
    }
    else
    {
        m_faults &= ~fault;
    }
}

void RobotNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Listen for Joystick messages
    RCLCPP_INFO(this->get_logger(), "Axes: [%f, %f, %f, %f, %f, %f]",
                msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], msg->axes[4], msg->axes[5]);
    RCLCPP_INFO(this->get_logger(), "Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
                msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3],
                msg->buttons[4], msg->buttons[5], msg->buttons[6], msg->buttons[7],
                msg->buttons[8], msg->buttons[9], msg->buttons[10], msg->buttons[11]);

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

RobotNode::~RobotNode()
{
}

void RobotNode::writeLogString(const std::string &msg)
{
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
}

void RobotNode::writeLog(const char *msg)
{
    RCLCPP_INFO(this->get_logger(), "%s", msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    g_myRobot = std::make_shared<Robot>();
    g_myRobotNode = std::make_shared<RobotNode>();
    rclcpp::spin(g_myRobotNode);
    rclcpp::shutdown();
    return 0;
}
