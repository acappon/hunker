#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "IMU_bno055.h"
#include "Motor.h"
#include "RobotNode.h"

std::shared_ptr<RobotNode> g_myRobot = nullptr;


RobotNode::RobotNode()
    : Node("robot_node")
{
    init();
}

void RobotNode::init()
{
    // Subscribe to joystick messages
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&RobotNode::joy_callback, this, std::placeholders::_1));

    pLwheel = new Motor(Motor::GPIO_L_WHEEL_DIR, Motor::GPIO_L_WHEEL_PWM, Motor::GPIO_L_WHEEL_COUNT);
    pRwheel = new Motor(Motor::GPIO_R_WHEEL_DIR, Motor::GPIO_R_WHEEL_PWM, Motor::GPIO_R_WHEEL_COUNT);
    pLknee = new Motor(Motor::GPIO_L_KNEE_DIR, Motor::GPIO_L_KNEE_PWM, Motor::GPIO_L_KNEE_COUNT);
    pRknee = new Motor(Motor::GPIO_R_KNEE_DIR, Motor::GPIO_R_KNEE_PWM, Motor::GPIO_R_KNEE_COUNT);
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
}

void RobotNode::writeLog(const std::string& msg)
{
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
}

void RobotNode::writeLog(const char* msg)
{
    RCLCPP_INFO(this->get_logger(), "%s", msg);
}

void RobotNode::estimateDeckOrientation()
{
}

bool RobotNode::isBalancingPossible()
{
    return false;
}

bool RobotNode::isFullyFolded()
{
    return false;
}

void RobotNode::periodic()
{
    imu.update();
    RCLCPP_INFO(this->get_logger(), "IMU up: [%f, %f, %f, %f]",
                imu.pos.up_pitch, imu.pos.up_roll, imu.pos.up_yaw, imu.pos.up_magnitude);
    RCLCPP_INFO(this->get_logger(), "IMU fall: [%f, %f, %f, %f]",
                imu.pos.fall_pitch, imu.pos.fall_roll, imu.pos.fall_yaw, imu.pos.fall_magnitude);

    if (!m_isEnabled)
    {
        disabledPeriodic();
    }
    else
    {
        estimateDeckOrientation();
        if (isBalancingPossible())
        {
            balancingPeriodic();
        }
        else
        {
            if (imu.pos.is_airborne)
            {
                airbornePeriodic();
            }
            else
            {
                if (isFullyFolded())
                {
                    m_isEnabled = false;
                }
                else
                {
                    landingPeriodic();
                }
            }
        }
    }
}

void RobotNode::disabledPeriodic()
{
    // TODO:  All motor power == 0
}

void RobotNode::balancingPeriodic()
{
    // TODO: Compare deck orientation to "up" vector, and adjust motor power to balance
}

void RobotNode::airbornePeriodic()
{
    // TODO: Move leg toward middle of range of motion, if there, motor power == 0
}

void RobotNode::landingPeriodic()
{
    // TODO: Move leg toward fully folded, if there, motor power == 0
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    g_myRobot = std::make_shared<RobotNode>();
    rclcpp::spin(g_myRobot);
    rclcpp::shutdown();
    return 0;
}
