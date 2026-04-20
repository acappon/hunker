
#include "common.h"

extern "C"
{
#include <lgpio.h>
}

extern std::shared_ptr<RobotNode> g_myRobotNode;

const MyGpio::GPIO_PIN Motor::m_gpio_for_enable[Motor::NUMBER_OF_MOTORS] = {
    MyGpio::GPIO_PIN::GPIO_R_WHEEL_ENABLE,
    MyGpio::GPIO_PIN::GPIO_L_WHEEL_ENABLE,
    MyGpio::GPIO_PIN::GPIO_R_KNEE_ENABLE,
    MyGpio::GPIO_PIN::GPIO_L_KNEE_ENABLE};

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

Motor::Motor()
{
}

Motor::~Motor()
{
}

int Motor::init()
{
    if (!g_myRobotNode->m_myGpio.initWheel(
            m_gpio_for_enable[RWheel],
            m_gpio_for_direction[RWheel],
            m_gpio_for_pwm[RWheel]))
    {
        g_myRobotNode->writeLog("Failed to claim GPIOs for right wheel motor");
    }

    if (!g_myRobotNode->m_myGpio.initWheel(
            m_gpio_for_enable[LWheel],
            m_gpio_for_direction[LWheel],
            m_gpio_for_pwm[LWheel]))
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
            m_gpio_for_direction[typ],
            m_gpio_for_pwm[typ],
            power > 0.0,
            abs(power)))
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
