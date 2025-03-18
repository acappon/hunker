#include <iostream>
#include <fstream>
#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "sensor_msgs/msg/joy.hpp"

#include "IMU_bno055.h"
#include "MyGpio.hpp"
#include "FaultIndicator.hpp"
#include "RobotNode.hpp"
#include "Motor.h"
#include "Robot.h"

extern std::shared_ptr<RobotNode> g_myRobotNode;

Robot::Robot()
{
    init();
}

void Robot::init()
{
    m_deck_pitch = 0.0;
    m_deck_roll = 0.0;
    m_myMotors.init();
}

Robot::~Robot()
{
}

static bool isRunning = true;
static bool isErrorMsgLogged = false;
void Robot::robotFunction()
{
    g_myRobotNode->m_faultIndicator.setFault(FaultIndicator::FAULT_WATCHDOG, isRunning);
    if (isRunning)
    {
        if (!isErrorMsgLogged)
        {
            g_myRobotNode->writeLog("Robot function did not finish in time");
            isErrorMsgLogged = true;
        }
    }
    isRunning = true;

    periodic();
    switch (m_robotState)
    {
    case ROBOT_STATE_DISABLED:
        disabledPeriodic();
        break;
    case ROBOT_STATE_BALANCING:
        balancingPeriodic();
        break;
    case ROBOT_STATE_AIRBORNE:
        airbornePeriodic();
        break;
    case ROBOT_STATE_LANDING:
        landingPeriodic();
        break;
    case ROBOT_STATE_HUNKERED:
        hunkeredPeriodic();
        break;
    default:
        g_myRobotNode->writeLog("Unknown robot state %d", m_robotState);
        g_myRobotNode->emergencyStop();
        break;
    }

    isRunning = false;
}

void Robot::estimateDeckOrientation()
{
}

bool Robot::isBalancingPossible()
{
    if ((imu.pos.is_airborne) || (m_robotState == ROBOT_STATE_LANDING) || (m_robotState == ROBOT_STATE_HUNKERED))
    {
        return false;
    }
    return true;
}

bool Robot::isFullyFolded()
{
    return false;
}

void Robot::periodic()
{
    imu.update();

    if (!g_myRobotNode->isRobotEnabled())
    {
        m_robotState = ROBOT_STATE_DISABLED;
    }
    else
    {
        estimateDeckOrientation();
        if (g_myRobotNode->m_joy_buttons[RobotNode::JOY_A] == 1)
        {
            m_robotState = ROBOT_STATE_LANDING;
        }

        if (isBalancingPossible())
        {
            m_robotState = ROBOT_STATE_BALANCING;
        }
        else
        {
            if (imu.pos.is_airborne)
            {
                m_robotState = ROBOT_STATE_AIRBORNE;
            }
            else
            {
                if (!isFullyFolded())
                {
                    m_robotState = ROBOT_STATE_LANDING;
                }
                else
                {
                    m_robotState = ROBOT_STATE_HUNKERED;
                }
            }
        }
    }
}

void Robot::disabledPeriodic()
{
    m_myMotors.setPower(Motor::MOTOR_TYPE::RWheel, 0.0);
    m_myMotors.setPower(Motor::MOTOR_TYPE::LWheel, 0.0);
    m_myMotors.setPower(Motor::MOTOR_TYPE::RKnee, 0.0);
    m_myMotors.setPower(Motor::MOTOR_TYPE::LKnee, 0.0);
}

void Robot::balancingPeriodic()
{
    // TODO: Compare deck orientation to "up" vector, and adjust motor power to balance
}

void Robot::airbornePeriodic()
{
    // TODO: Move leg toward middle of range of motion, if there, motor power == 0
}

void Robot::landingPeriodic()
{
    // TODO: Move leg toward fully folded, if there, motor power == 0
}

void Robot::hunkeredPeriodic()
{
    // TODO: Maintain sufficient power to keep springs from extending legs
    // Probably zero if upriight, but will need power if upside down
}
