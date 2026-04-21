
#include "common.h"

extern "C"
{
#include <lgpio.h>
}

extern std::shared_ptr<RobotNode> g_myRobotNode;

MyGpio::MyGpio()
{
}

MyGpio::~MyGpio()
{
    m_lgpio_chip = lgGpiochipClose(m_lgpio_chip);
}

bool MyGpio::initEnableAndFaultLED()
{
    int iRet = -999;

    // Open the GPIO chip
    int chipnum = 4; // GPIO chip #4 is the one for Raspberry Pi 5
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

    iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_PULL_NONE, GPIO_PIN::GPIO_LED_ENABLE, 0);
    if (iRet < 0)
    {
        g_myRobotNode->writeLog("Failed to claim GPIO for enable LED line");
    }

    iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_PULL_NONE, GPIO_PIN::GPIO_LED_FAULT, 0);
    if (iRet < 0)
    {
        g_myRobotNode->writeLog("Failed to claim GPIO fault line");
    }

    return (iRet == 0);
}

bool MyGpio::isValidPin(MyGpio::GPIO_PIN pin)
{
    if (pin < 0 || pin > 27)
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
        if (iRet != 0)
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
    if (!gpioWrite(GPIO_PIN::GPIO_LED_ENABLE, state))
    {
        g_myRobotNode->writeLog("Failed to write to enable LED GPIO");
    }
}

void MyGpio::setFaultLED(bool state)
{
    if (!gpioWrite(GPIO_PIN::GPIO_LED_FAULT, state))
    {
        g_myRobotNode->writeLog("Failed to write to fault LED GPIO");
    }
}

void MyGpio::enableWheelMotors(bool isEnabled)
{
 
 
 
}

bool MyGpio::setMotorPower(MyGpio::GPIO_PIN dirPin, MyGpio::GPIO_PIN pwmPin, bool isFwd, double power)
{

    return true;
}
