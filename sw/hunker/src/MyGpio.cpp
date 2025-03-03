
extern "C"
{
#include <lgpio.h>
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "sensor_msgs/msg/joy.hpp"

#include "MyGpio.hpp"
#include "FaultIndicator.hpp"
#include "RobotNode.hpp"

extern std::shared_ptr<RobotNode> g_myRobotNode;

MyGpio::MyGpio()
{
}

MyGpio::~MyGpio()
{
    m_lgpio_chip = lgGpiochipClose(m_lgpio_chip);
}

bool MyGpio::init()
{
    bool bRet = true;

    // Open the GPIO chip
    int chipnum = 5;
    m_lgpio_chip = lgGpiochipOpen(chipnum);
    if (m_lgpio_chip < 0)
    {
        std::string msg =
            "Failed to open GPIO chip:  " + std::to_string(chipnum) + +"   " + std::to_string(m_lgpio_chip);
        g_myRobotNode->writeLog(msg);
        bRet = false;
    }
    else
    {
        std::string msg =
            "Opened GPIO chip:  " + std::to_string(chipnum) + +"   " + std::to_string(m_lgpio_chip);
        g_myRobotNode->writeLog(msg);
        bRet = false;
    }

    if (lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, GPIO_PIN::GPIO_TEST, 0))
    {
        g_myRobotNode->writeLog("Failed to claim GPIO test line");
        bRet = false;
    }

    if (lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, GPIO_PIN::GPIO_LED_ENABLE, 0))
    {
        g_myRobotNode->writeLog("Failed to claim GPIO enable line");
        bRet = false;
    }

    if (lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, GPIO_PIN::GPIO_LED_FAULT, 0))
    {
        g_myRobotNode->writeLog("Failed to claim GPIO fault line");
        bRet = false;
    }

    return bRet;
}

bool MyGpio::initWheel(MyGpio::GPIO_PIN enable, MyGpio::GPIO_PIN dir, MyGpio::GPIO_PIN pwm, MyGpio::GPIO_PIN count)
{
    bool bRet = true;

    if (lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, enable, 0))
    {
        g_myRobotNode->writeLog("Failed to claim enable GPIO line");
        bRet = false;
    }

    if (lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, dir, 0))
    {
        g_myRobotNode->writeLog("Failed to claim direction GPIO line");
        bRet = false;
    }

    if (lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, pwm, 0))
    {
        g_myRobotNode->writeLog("Failed to claim PWM GPIO line");
        bRet = false;
    }

    if (lgGpioClaimInput(m_lgpio_chip, LG_SET_ACTIVE_LOW, count))
    {
        g_myRobotNode->writeLog("Failed to claim count GPIO line");
        bRet = false;
    }

    return bRet;
}

bool MyGpio::gpioWrite(MyGpio::GPIO_PIN pin, bool value)
{
    if (!lgGpioWrite(m_lgpio_chip, pin, value ? 1 : 0))
    {
        return false;
    }
    return true;
}

bool MyGpio::gpioRead(MyGpio::GPIO_PIN pin)
{
    int value;
    if (!lgGpioRead(m_lgpio_chip, pin))
    {
        return false;
    }
    return value;
}

void MyGpio::setEnableLED(bool state)
{
    if (!gpioWrite(GPIO_PIN::GPIO_LED_ENABLE, state ? 1 : 0))
    {
        g_myRobotNode->writeLog("Failed to write to enable LED GPIO");
    }
}

void MyGpio::setFaultLED(bool state)
{
    g_myRobotNode->writeLog("Fault LED state: " + (state ? std::string("XXX") : std::string("   ")));

     if (!gpioWrite(GPIO_PIN::GPIO_LED_FAULT, state ? 1 : 0))
    {
        g_myRobotNode->writeLog("Failed to write to fault LED GPIO");
    }
}

bool MyGpio::setMotorPower(MyGpio::GPIO_PIN enablePin, MyGpio::GPIO_PIN dirPin, MyGpio::GPIO_PIN pwmPin, bool isEnabled, bool isFwd, double power)
{
    // Set enable
    if (!gpioWrite(enablePin, isEnabled ? 1 : 0))
    {
        g_myRobotNode->writeLog("Failed to write to enable GPIO");
        return false;
    }

    // Set direction
    if (!gpioWrite(dirPin, isFwd ? 1 : 0))
    {
        g_myRobotNode->writeLog("Failed to write to direction GPIO");
        return false;
    }

    // Set PWM
    int pwmDutyCycle = abs(power) * 100;
    int ret = lgTxPwm(m_lgpio_chip, pwmPin, 10000, pwmDutyCycle, 0, 0);
    if (ret < 0)
    {
        g_myRobotNode->writeLog("Failed to set PWM duty cycle");
        return false;
    }

    return true;
}
