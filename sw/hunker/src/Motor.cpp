
#include <iostream>
#include <fstream>

#include <memory>
#include <csignal>

extern "C"
{
#include <lgpio.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "IMU_bno055.h"
#include "MyGpio.hpp"
#include "Motor.h"
#include "Robot.h"
#include "RobotNode.hpp"

extern std::shared_ptr<RobotNode> g_myRobotNode;

const MyGpio::GPIO_PIN Motor::m_gpio_for_enable[Motor::NUMBER_OF_MOTORS] = {
    MyGpio::GPIO_PIN::GPIO_R_WHEEL_ENABLE,
    MyGpio::GPIO_PIN::GPIO_L_WHEEL_ENABLE,
    MyGpio::GPIO_PIN::GPIO_R_KNEE_ENABLE,
    MyGpio::GPIO_PIN::GPIO_L_KNEE_ENABLE};

const MyGpio::GPIO_PIN Motor::m_gpio_for_turn_count[Motor::NUMBER_OF_MOTORS] = {
    MyGpio::GPIO_PIN::GPIO_R_WHEEL_COUNT,
    MyGpio::GPIO_PIN::GPIO_L_WHEEL_COUNT,
    MyGpio::GPIO_PIN::GPIO_R_KNEE_COUNT,
    MyGpio::GPIO_PIN::GPIO_L_KNEE_COUNT};

const MyGpio::GPIO_PIN Motor::m_gpio_for_pwm[Motor::NUMBER_OF_MOTORS] = {
    MyGpio::GPIO_PIN::GPIO_R_WHEEL_PWM,
    MyGpio::GPIO_PIN::GPIO_L_WHEEL_PWM,
    MyGpio::GPIO_PIN::GPIO_R_KNEE_PWM,
    MyGpio::GPIO_PIN::GPIO_L_KNEE_PWM};

const MyGpio::GPIO_PIN Motor::m_gpio_for_direction[Motor::NUMBER_OF_MOTORS] = {
    MyGpio::GPIO_PIN::GPIO_R_WHEEL_DIR,
    MyGpio::GPIO_PIN::GPIO_L_WHEEL_DIR,
    MyGpio::GPIO_PIN::GPIO_R_KNEE_DIR,
    MyGpio::GPIO_PIN::GPIO_L_KNEE_DIR};

double Motor::m_power[Motor::NUMBER_OF_MOTORS] = {0.0};
int Motor::m_turn_count[Motor::NUMBER_OF_MOTORS] = {0};

void turnCountISR(int num_alerts, lgGpioAlert_p alerts, void *user)
{
    Motor::MOTOR_TYPE typ = Motor::RWheel;
    switch (alerts->report.gpio)
    {
    case MyGpio::GPIO_PIN::GPIO_R_WHEEL_COUNT:
        typ = Motor::RWheel;
        break;
    case MyGpio::GPIO_PIN::GPIO_L_WHEEL_COUNT:
        typ = Motor::LWheel;
        break;
    case MyGpio::GPIO_PIN::GPIO_R_KNEE_COUNT:
        typ = Motor::RKnee;
        break;
    case MyGpio::GPIO_PIN::GPIO_L_KNEE_COUNT:
        typ = Motor::LKnee;
        break;
    default:
        throw;
    }

    for (int i = 0; i < num_alerts; ++i)
    {
        if (alerts->report.level == 1) /* 0=low, 1=high, 2=watchdog */
        {
            if (Motor::isForward(typ))
            {
                Motor::m_turn_count[typ]++;
            }
            else
            {
                Motor::m_turn_count[typ]--;
            }
        }
    }
}

Motor::Motor()
{
    init();
}

Motor::~Motor()
{
}

int Motor::init()
{
    if (!g_myRobotNode->m_myGpio.initWheel(
            m_gpio_for_enable[RWheel],
            m_gpio_for_direction[RWheel],
            m_gpio_for_pwm[RWheel],
            m_gpio_for_turn_count[RWheel]))
    {
        g_myRobotNode->writeLog("Failed to claim GPIOs for right wheel motor");
    }

    if (!g_myRobotNode->m_myGpio.initWheel(
            m_gpio_for_enable[LWheel],
            m_gpio_for_direction[LWheel],
            m_gpio_for_pwm[LWheel],
            m_gpio_for_turn_count[LWheel]))
    {
        g_myRobotNode->writeLog("Failed to claim GPIOs for left wheel motor");
    }

    return 0;
}

std::string Motor::motorName(Motor::MOTOR_TYPE typ)
{
    switch (typ)
    {
    case RWheel:
        return "RWheel";
    case LWheel:
        return "LWheel";
    case RKnee:
        return "RKnee";
    case LKnee:
        return "LKnee";
    default:
        return "Unknown";
    }
}

void Motor::setPower(Motor::MOTOR_TYPE typ, double power) // -1.0 to 1.0
{
    m_power[typ] = power;

    if (!g_myRobotNode->m_myGpio.setMotorPower(
            m_gpio_for_enable[typ],
            m_gpio_for_direction[typ],
            m_gpio_for_pwm[typ],
            power != 0.0,
            power > 0.0,
            power))
    {
        g_myRobotNode->writeLog("Failed to set power for " + motorName(typ));
    }
}

bool Motor::isEnabled(Motor::MOTOR_TYPE typ)
{
    return m_power[typ] != 0.0;
}

bool Motor::isForward(Motor::MOTOR_TYPE typ)
{
    return m_power[typ] > 0.0;
}

double Motor::getPower(Motor::MOTOR_TYPE typ)
{
    return m_power[typ];
}

int Motor::getTurnCount(Motor::MOTOR_TYPE typ)
{
    return m_turn_count[typ];
}
