
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
    int chipnum = 4; // GPIO chip #4 is the one for Raspberry Pi 5
    m_lgpio_chip = lgGpiochipOpen(chipnum);
    if (m_lgpio_chip < 0)
    {
        g_myRobotNode->writeLog("Failed to open GPIO chip: %d  error: %d", chipnum, m_lgpio_chip);
        return false;
    }
    g_myRobotNode->writeLog("Opened GPIO chip: %d  handle: %d", chipnum, m_lgpio_chip);

    bool success = true;

    int iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_PULL_NONE, GPIO_PIN::GPIO_LED_ENABLE, 0);
    if (iRet < 0)
    {
        g_myRobotNode->writeLog("Failed to claim GPIO for enable LED line: %s", lguErrorText(iRet));
        success = false;
    }

    iRet = lgGpioClaimOutput(m_lgpio_chip, LG_SET_PULL_NONE, GPIO_PIN::GPIO_LED_FAULT, 0);
    if (iRet < 0)
    {
        g_myRobotNode->writeLog("Failed to claim GPIO for fault LED line: %s", lguErrorText(iRet));
        success = false;
    }

    return success;
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
            std::string msg = "Failed to write to GPIO pin" + std::to_string(pin) + " ";
            switch(iRet)
            {
                case LG_GPIO_BUSY:
                    msg += "LG_GPIO_BUSY";
                    break;

                default:
                    msg += std::to_string(iRet) + "  " + lguErrorText(iRet);
                    break;
            }
            g_myRobotNode->writeLog(msg);
        }
    }
    return (iRet == 0);
}

bool MyGpio::gpioRead(MyGpio::GPIO_PIN pin)
{
    if (!isValidPin(pin))
    {
        g_myRobotNode->writeLog("gpioRead: invalid pin %d", pin);
        return false;
    }
    int value = lgGpioRead(m_lgpio_chip, pin);
    if (value < 0)
    {
        g_myRobotNode->writeLog("gpioRead failed on pin %d: %s", pin, lguErrorText(value));
        return false;
    }
    return (value != 0);
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
