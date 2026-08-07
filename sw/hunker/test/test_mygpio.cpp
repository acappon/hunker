#include <gtest/gtest.h>
#include "common.h"
#include "stubs.h"

extern std::shared_ptr<RobotNode> g_myRobotNode;

class MyGpioTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        stubs::reset();
        g_myRobotNode = std::make_shared<RobotNode>();
    }

    void TearDown() override
    {
        g_myRobotNode.reset();
    }

    MyGpio m_gpio;
};

// --- isValidPin ---

TEST_F(MyGpioTest, ValidPinEnableLED)
{
    EXPECT_TRUE(m_gpio.isValidPin(MyGpio::GPIO_LED_ENABLE));
}

TEST_F(MyGpioTest, ValidPinFaultLED)
{
    EXPECT_TRUE(m_gpio.isValidPin(MyGpio::GPIO_LED_FAULT));
}

TEST_F(MyGpioTest, InvalidPinNegative)
{
    EXPECT_FALSE(m_gpio.isValidPin(static_cast<MyGpio::GPIO_PIN>(-1)));
}

TEST_F(MyGpioTest, InvalidPinTooHigh)
{
    EXPECT_FALSE(m_gpio.isValidPin(static_cast<MyGpio::GPIO_PIN>(28)));
}

TEST_F(MyGpioTest, BoundaryPin0Valid)
{
    EXPECT_TRUE(m_gpio.isValidPin(static_cast<MyGpio::GPIO_PIN>(0)));
}

TEST_F(MyGpioTest, BoundaryPin27Valid)
{
    EXPECT_TRUE(m_gpio.isValidPin(static_cast<MyGpio::GPIO_PIN>(27)));
}

// --- gpioWrite ---

TEST_F(MyGpioTest, GpioWriteSuccess)
{
    m_gpio.initEnableAndFaultLED();
    EXPECT_TRUE(m_gpio.gpioWrite(MyGpio::GPIO_LED_ENABLE, true));
    EXPECT_EQ(stubs::lastGpio(), MyGpio::GPIO_LED_ENABLE);
    EXPECT_EQ(stubs::lastValue(), 1);
}

TEST_F(MyGpioTest, GpioWriteFalse)
{
    m_gpio.initEnableAndFaultLED();
    EXPECT_TRUE(m_gpio.gpioWrite(MyGpio::GPIO_LED_FAULT, false));
    EXPECT_EQ(stubs::lastGpio(), MyGpio::GPIO_LED_FAULT);
    EXPECT_EQ(stubs::lastValue(), 0);
}

TEST_F(MyGpioTest, GpioWriteFailureReturnsError)
{
    m_gpio.initEnableAndFaultLED();
    stubs::setWriteFail(true);
    EXPECT_FALSE(m_gpio.gpioWrite(MyGpio::GPIO_LED_ENABLE, true));
}

// --- setEnableLED / setFaultLED ---

TEST_F(MyGpioTest, SetEnableLEDOn)
{
    m_gpio.initEnableAndFaultLED();
    m_gpio.setEnableLED(true);
    EXPECT_EQ(stubs::lastGpio(), MyGpio::GPIO_LED_ENABLE);
    EXPECT_EQ(stubs::lastValue(), 1);
}

TEST_F(MyGpioTest, SetFaultLEDOff)
{
    m_gpio.initEnableAndFaultLED();
    m_gpio.setFaultLED(false);
    EXPECT_EQ(stubs::lastGpio(), MyGpio::GPIO_LED_FAULT);
    EXPECT_EQ(stubs::lastValue(), 0);
}

// --- initEnableAndFaultLED ---

TEST_F(MyGpioTest, InitReturnsTrue)
{
    EXPECT_TRUE(m_gpio.initEnableAndFaultLED());
}

// --- enableWheelMotors (currently a no-op but should not crash) ---

TEST_F(MyGpioTest, EnableWheelMotorsDoesNotCrash)
{
    m_gpio.enableWheelMotors(true);
    m_gpio.enableWheelMotors(false);
}
