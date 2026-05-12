#include "common.h"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "sensor_msgs/msg/joy.hpp"

extern std::shared_ptr<RobotNode> g_myRobotNode;

Robot::Robot()
{
}

void Robot::init()
{
    m_driveType = TANK_ROBOT_FRAME;
    m_robotState = ROBOT_STATE_DISABLED;
    m_deck_pitch = 0.0;
    m_deck_roll = 0.0;
 
    m_myBalanceDrive.init();
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

std::string Robot::getRobotStateText()
{
    switch (m_robotState)
    {
    case ROBOT_STATE_DISABLED:
        return "Disabled";
    case ROBOT_STATE_BALANCING:
        return "Balancing";
    case ROBOT_STATE_AIRBORNE:
        return "Airborne";
    case ROBOT_STATE_LANDING:
        return "Landing";
    case ROBOT_STATE_HUNKERED:
        return "Hunkered";
    default:
        return "Unknown";
    }
}

void Robot::estimateDeckOrientation()
{
}

bool Robot::isBalancingPossible()
{
    // TODO:  After estimating deck orientation, robot is in a position that can't reach balance.
    //        For example, legs extended way out but laying flat on ground.
    //        For example, HUNKERED down, but upside down on the ground
    //        The idea is that instead of the wheels going nuts trying to balance, the robot will seek
    //        HUNKER position, and doing so might even flip it rightside up, (if the body is lighter than the legs)

    return true;
}

bool Robot::isFullyFolded()
{
    // TODO:  There will be limit switches at both extremes of leg position, or if not, at least an encoder position
    return false;
}

void Robot::periodic()
{
    if (!g_myRobotNode->isRobotEnabled())
    {
        m_robotState = ROBOT_STATE_DISABLED;
    }
    else
    {
        estimateDeckOrientation();
        if (false)    //g_myRobotNode->m_imu.isAirborne())
        {
            m_robotState = ROBOT_STATE_AIRBORNE;
        }
        else if ((m_robotState == ROBOT_STATE_LANDING) || (m_robotState == ROBOT_STATE_HUNKERED))
        {
            if ((g_myRobotNode->m_joy_buttons[RobotNode::JOY_Y] == 1) && (isBalancingPossible()))
            {
                m_robotState = ROBOT_STATE_BALANCING;
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
        else // BALANCING
        {
            if ((g_myRobotNode->m_joy_buttons[RobotNode::JOY_A] == 1) || (!isBalancingPossible()))
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
            else
            {
                m_robotState = ROBOT_STATE_BALANCING;
            }
        }
    }
}

void Robot::disabledPeriodic()
{
    m_myBalanceDrive.setEnable(false);
}

void Robot::balancingPeriodic()
{
    m_myBalanceDrive.setEnable(true);

    double power_R = 0.0; // Range -1.0 to 1.0, joystick contribution only
    double power_L = 0.0;

    double joyLeft_FB  = g_myRobotNode->m_joy_axes[RobotNode::LJOY_FWD_BACK];
    double joyRight_FB = g_myRobotNode->m_joy_axes[RobotNode::RJOY_FWD_BACK];
    double joyRight_LR = g_myRobotNode->m_joy_axes[RobotNode::RJOY_LEFT_RIGHT];

    if (m_driveType == TANK_ROBOT_FRAME)
    {
        power_R += joyRight_FB;
        power_L += joyLeft_FB;
    }
    else if (m_driveType == ARCADE_ROBOT_FRAME)
    {
        power_R += joyRight_FB + joyRight_LR;
        power_L += joyRight_FB - joyRight_LR;
    }
    else if (m_driveType == ARCADE_FIELD_FRAME)
    {
        const double joystickWeight = 0.2;
        power_R += joyRight_FB * joystickWeight + joyRight_LR * joystickWeight;
        power_L += joyRight_FB * joystickWeight - joyRight_LR * joystickWeight;

        g_myRobotNode->writeLog("Field frame driving not yet implemented");
        g_myRobotNode->emergencyStop();
    }
    else
    {
        g_myRobotNode->writeLog("Unknown drive type %d", m_driveType);
        g_myRobotNode->emergencyStop();
    }

    // NOTE: Left motor inversion is handled inside BalanceDrive::update().
    // Do NOT invert power_L here — pass raw joystick intent to update().

    // Clamp joystick contribution before passing to update()
    if (power_L >  1.0) power_L =  1.0;
    if (power_L < -1.0) power_L = -1.0;
    if (power_R >  1.0) power_R =  1.0;
    if (power_R < -1.0) power_R = -1.0;

    m_myBalanceDrive.update(power_L, power_R);
}

void Robot::airbornePeriodic()
{
    m_myBalanceDrive.setEnable(true);
    // TODO: Move leg toward middle of range of motion, if there, motor power == 0
}

void Robot::landingPeriodic()
{
     m_myBalanceDrive.setEnable(true);
   // TODO: Move leg toward fully folded, if there, motor power == 0
}

void Robot::hunkeredPeriodic()
{
     m_myBalanceDrive.setEnable(true);
   // TODO: Maintain sufficient power to keep springs from extending legs
    // Probably zero if upriight, but will need power if upside down
}
