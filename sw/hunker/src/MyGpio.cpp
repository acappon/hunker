
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
#include "Motor.h"
#include "IMU_bno055.h"
#include "Robot.h"
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
    int iRet = -999;

    // Open the GPIO chip
    int chipnum = 4;   // GPIO chip #4 is the one for Raspberry Pi 5
    m_lgpio_chip = lgGpiochipOpen(chipnum);
    if (m_lgpio_chip < 0)
    {
        std::string msg =
            "Failed to open GPIO chip:  " + std::to_string(chipnum) + +"   " + std::to_string(m_lgpio_chip);
        g_myRobotNode->writeLog(msg);
        return false;
    }
    else
    {
        std::string msg =
            "Opened GPIO chip:  " + std::to_string(chipnum) + +"   " + std::to_string(m_lgpio_chip);
        g_myRobotNode->writeLog(msg);
        iRet = 0;
    }

    iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_PULL_UP, GPIO_PIN::GPIO_LED_ENABLE, 0);
    if (iRet < 0)
    {
        g_myRobotNode->writeLog("Failed to claim GPIO for enable LED line");
    }

    iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_PULL_UP, GPIO_PIN::GPIO_LED_FAULT, 0);
    if (iRet < 0)
    {
        g_myRobotNode->writeLog("Failed to claim GPIO fault line");
    }

    return (iRet == 0);
}

bool MyGpio::initWheel(MyGpio::GPIO_PIN enable, MyGpio::GPIO_PIN dir, MyGpio::GPIO_PIN pwm, MyGpio::GPIO_PIN count)
{
    int iRet = -999;

    if (isValidPin(enable))
    {
        iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, enable, 0);
        if (iRet < 0)
        {
            g_myRobotNode->writeLog("Failed to claim GPIO for wheel enable line");
        }
    }

    if (isValidPin(dir))
    {
        iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, dir, 0);
        {
            g_myRobotNode->writeLog("Failed to claim direction GPIO line");
        }
    }

    if (isValidPin(pwm))
    {
        iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_ACTIVE_LOW, pwm, 0);
        if (iRet < 0)
        {
            g_myRobotNode->writeLog("Failed to claim PWM GPIO line");
        }
    }

    if (isValidPin(count))
    {
        iRet = lgGpioClaimInput(m_lgpio_chip, LG_SET_ACTIVE_LOW, count);
        if (iRet < 0)
        {
            g_myRobotNode->writeLog("Failed to claim count GPIO line");
        }
    }

    return iRet;
}

bool MyGpio::isValidPin(MyGpio::GPIO_PIN pin)
{
    if( pin < 0 || pin > 26)
    {
        return false;
    }
    return true;
}

bool MyGpio::gpioWrite(MyGpio::GPIO_PIN pin, bool value)
{
    int iRet = -999;
    if (isValidPin(pin))
    {
        iRet = lgGpioWrite(m_lgpio_chip, pin, value ? 1 : 0);
        if(iRet != 0)
        {
            g_myRobotNode->writeLog("Failed to write to GPIO pin %d , iRet = %s", pin, lguErrorText(iRet));
        }
    }
    return (iRet == 0);
}

bool MyGpio::gpioRead(MyGpio::GPIO_PIN pin)
{
    int value = 0;
    if (isValidPin(pin))
    {
        value = lgGpioRead(m_lgpio_chip, pin);
    }
    return value;
}

void MyGpio::setEnableLED(bool state)
{
    bool not_state = !state;   // To turn on the LED, set GPIO to 0

    if (!gpioWrite(GPIO_PIN::GPIO_LED_ENABLE, not_state))
    {
        g_myRobotNode->writeLog("Failed to write to enable LED GPIO");
    }
}

void MyGpio::setFaultLED(bool state)
{
    bool not_state = !state;   // To turn on the LED, set GPIO to 0

    if (!gpioWrite(GPIO_PIN::GPIO_LED_FAULT, not_state))
    {
        g_myRobotNode->writeLog("Failed to write to fault LED GPIO");
    }
}

void MyGpio::enableWheelMotors(bool isEnabled)
{
    if (!gpioWrite(GPIO_PIN::GPIO_R_WHEEL_ENABLE, isEnabled))
    {
        g_myRobotNode->writeLog("Failed to write to right wheel enable GPIO");
    }

    if (!gpioWrite(GPIO_PIN::GPIO_L_WHEEL_ENABLE, isEnabled))
    {
        g_myRobotNode->writeLog("Failed to write to left wheel enable GPIO");
    }
}

bool MyGpio::setMotorPower(MyGpio::GPIO_PIN dirPin, MyGpio::GPIO_PIN pwmPin, bool isFwd, double power)
{
    // Set direction
    if (!gpioWrite(dirPin, isFwd))
    {
        g_myRobotNode->writeLog("Failed to write to direction GPIO");
        return false;
    }

    // Set PWM
    int pwmDutyCycle = abs(power) * 100;
    int iRet = lgTxPwm(m_lgpio_chip, pwmPin, 10000, pwmDutyCycle, 0, 0);
    if (iRet < 0)
    {
        g_myRobotNode->writeLog("Failed to set PWM duty cycle");
        return false;
    }

    return true;
}
