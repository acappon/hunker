// Stub implementations for hardware and ROS2 dependencies
// so the hunker unit tests can compile and link without real hardware.

#include "common.h"

extern "C" {
#include "lgpio.h"
}

// Global RobotNode pointer used by production sources
std::shared_ptr<RobotNode> g_myRobotNode = nullptr;

// ---------- RobotNode stubs ----------
// The production RobotNode.cpp pulls in too many ROS2 specifics,
// so we provide minimal stubs for the methods that other modules call.

RobotNode::RobotNode()
    : Node("RobotNode"), m_isControllerConnected(false), m_isRobotEnabled(false)
{
    m_isRobotEmergencyStopped = false;
    for (int i = 0; i < NUMBER_OF_JOY_AXES; ++i) m_joy_axes[i] = 0.0;
    for (int i = 0; i < NUMBER_OF_JOY_BUTTONS; ++i) m_joy_buttons[i] = false;
    m_imu_orientation.fill(0.0);
    m_imu_linear_acceleration.fill(0.0);
    m_imu_angular_velocity.fill(0.0);
}

RobotNode::~RobotNode() {}

void RobotNode::init() {}

void RobotNode::writeLog(const std::string &, ...) {}

bool RobotNode::isRobotEnabled() { return m_isRobotEnabled; }

void RobotNode::emergencyStop()
{
    m_isRobotEmergencyStopped = true;
    m_isRobotEnabled = false;
}

// ---------- lgpio stubs ----------
extern "C"
{
    static int g_lgpio_stub_chip = 42;
    static int g_lgpio_last_gpio = -1;
    static int g_lgpio_last_value = -1;
    static bool g_lgpio_write_fail = false;

    int lgGpiochipOpen(int /*chipnum*/) { return g_lgpio_stub_chip; }
    int lgGpiochipClose(int /*handle*/) { return 0; }
    int lgGpioClaimOutput(int /*handle*/, int /*flags*/, int /*gpio*/, int /*level*/) { return 0; }

    int lgGpioWrite(int /*handle*/, int gpio, int level)
    {
        g_lgpio_last_gpio = gpio;
        g_lgpio_last_value = level;
        return g_lgpio_write_fail ? LG_GPIO_BUSY : 0;
    }

    int lgGpioRead(int /*handle*/, int gpio)
    {
        (void)gpio;
        return g_lgpio_last_value;
    }

    const char *lguErrorText(int /*error*/) { return "stub error"; }
}

// Accessors so tests can inspect/control the stubs
namespace stubs
{
    int lastGpio() { return g_lgpio_last_gpio; }
    int lastValue() { return g_lgpio_last_value; }
    void setWriteFail(bool fail) { g_lgpio_write_fail = fail; }
    void reset()
    {
        g_lgpio_last_gpio = -1;
        g_lgpio_last_value = -1;
        g_lgpio_write_fail = false;
    }
} // namespace stubs
