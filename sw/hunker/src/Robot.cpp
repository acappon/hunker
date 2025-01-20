#include <iostream>
#include <fstream>
#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "IMU_bno055.h"
#include "Motor.h"
#include "Robot.h"
#include "RobotNode.h"

extern std::shared_ptr<RobotNode> g_myRobotNode;

Robot::Robot()
{
    init();
}

void Robot::init()
{
    pMotors = new Motor();
}

Robot::~Robot()
{
    delete pMotors;
}

static bool isRunning = true;
static bool isErrorMsgLogged = false;
void Robot::robotFunction()
{
    g_myRobotNode->setFault(RobotNode::FAULT_ROBOT_WATCHDOG, isRunning);
    if(isRunning)
    {
        if(!isErrorMsgLogged)
        {
            g_myRobotNode->writeLog("Robot function did not finish in time");
            isErrorMsgLogged = true;
        }
        return;
    }
    isRunning = true;
    periodic();
    isRunning = false;
}

void Robot::estimateDeckOrientation()
{
}

bool Robot::isBalancingPossible()
{
    return false;
}

bool Robot::isFullyFolded()
{
    return false;
}

void Robot::periodic()
{
    imu.update();

    if (!g_myRobotNode->isEnabled())
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
                    g_myRobotNode->setEnabled(false);
                }
                else
                {
                    landingPeriodic();
                }
            }
        }
    }
}

void Robot::disabledPeriodic()
{
    // TODO:  All motor power == 0
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
