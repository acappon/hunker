
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

extern std::shared_ptr<RobotNode> g_myRobotNode;

const int Motor::m_gpio_for_turn_count[Motor::NUMBER_OF_MOTORS] = {
    GPIO_PIN::GPIO_R_WHEEL_COUNT,
    GPIO_PIN::GPIO_L_WHEEL_COUNT,
    GPIO_PIN::GPIO_R_KNEE_COUNT,
    GPIO_PIN::GPIO_L_KNEE_COUNT};

const int Motor::m_gpio_for_pwm[Motor::NUMBER_OF_MOTORS] = {
    GPIO_PIN::GPIO_R_WHEEL_PWM,
    GPIO_PIN::GPIO_L_WHEEL_PWM,
    GPIO_PIN::GPIO_R_KNEE_PWM,
    GPIO_PIN::GPIO_L_KNEE_PWM,
};

const int Motor::m_gpio_for_direction[Motor::NUMBER_OF_MOTORS] = {
    GPIO_PIN::GPIO_R_WHEEL_DIR,
    GPIO_PIN::GPIO_L_WHEEL_DIR,
    GPIO_PIN::GPIO_R_KNEE_DIR,
    GPIO_PIN::GPIO_L_KNEE_DIR,
};

double Motor::m_power[Motor::NUMBER_OF_MOTORS] = {0.0};
sig_atomic_t Motor::m_turn_count[Motor::NUMBER_OF_MOTORS] = {0};

void turnCountISR(int num_alerts, lgGpioAlert_p alerts, void *user)
{
    Motor::MOTOR_TYPE typ = Motor::RWheel;
    switch (alerts->report.gpio)
    {
    case GPIO_PIN::GPIO_R_WHEEL_COUNT:
        typ = Motor::RWheel;
        break;
    case GPIO_PIN::GPIO_L_WHEEL_COUNT:
        typ = Motor::LWheel;
        break;
    case GPIO_PIN::GPIO_R_KNEE_COUNT:
        typ = Motor::RKnee;
        break;
    case GPIO_PIN::GPIO_L_KNEE_COUNT:
        typ = Motor::LKnee;
        break;
    default:
        throw;
    }

    for (int i = 0; i < num_alerts; ++i)
    {
        if (alerts->report.level == 1) /* 0=low, 1=high, 2=watchdog */
        {
            if (Motor::isReverse(typ))
            {
                Motor::m_turn_count[typ]--;
            }
            else
            {
                Motor::m_turn_count[typ]++;
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
    lgGpiochipClose(g_myRobotNode->m_lgpio_chip);
}

int Motor::init()
{
    // Prepare interrupt service for turn count GPIO pin
    for (int i = 0; i < NUMBER_OF_MOTORS; i++)
    {
        if (lgGpioClaimInput(g_myRobotNode->m_lgpio_chip, 0, m_gpio_for_turn_count[i]))
        {
            g_myRobotNode->writeLog("Failed to claim turn count GPIO lines");
            return 1;
        }

        if (lgGpioClaimOutput(g_myRobotNode->m_lgpio_chip, 0, m_gpio_for_pwm[i], 0))
        {
            g_myRobotNode->writeLog("Failed to claim PWM GPIO lines");
            return 1;
        }

        if (lgGpioClaimOutput(g_myRobotNode->m_lgpio_chip, 0, m_gpio_for_direction[i], 0))
        {
            g_myRobotNode->writeLog("Failed to claim Direction GPIO lines");
            return 1;
        }

        m_power[i] = 0.0;
        m_turn_count[i] = 0;
    }
    return 0;
}

void Motor::setPower(Motor::MOTOR_TYPE typ, double power) // -1.0 to 1.0
{
    m_power[typ] = power;

    // Set direction
    lgGpioWrite(g_myRobotNode->m_lgpio_chip, m_gpio_for_direction[typ], isReverse(typ) ? 1 : 0);

    int pwmDutyCycle = abs(m_power[typ]) * 100;
    int ret = lgTxPwm(g_myRobotNode->m_lgpio_chip, m_gpio_for_pwm[typ], 10000, pwmDutyCycle, 0, 0);
    if (ret < 0)
    {
        g_myRobotNode->writeLog("Failed to set PWM duty cycle");
    }
}

double Motor::getPower(Motor::MOTOR_TYPE typ)
{
    return m_power[typ];
}

bool Motor::isReverse(Motor::MOTOR_TYPE typ)
{
    return m_power[typ] < 0;
}

int Motor::getTurnCount(Motor::MOTOR_TYPE typ)
{
    return m_turn_count[typ];
}
