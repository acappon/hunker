#include "common.h"

#include <algorithm>

extern "C"
{
#include <lgpio.h>
}

extern std::shared_ptr<RobotNode> g_myRobotNode;

BalanceDrive::BalanceDrive()
{
}

BalanceDrive::~BalanceDrive()
{
}

int BalanceDrive::init()
{
    // Controller configuration (motor type, idle mode, CAN ID) is done once in
    // REV Hardware Client 2. Nothing is written to the SPARKs here, and flash is never burned.
    try
    {
        m_bus = std::make_unique<spark::CanBus>("can0");
        m_left = std::make_unique<spark::SparkMax>(*m_bus, kLeftMotorCanID);
        m_right = std::make_unique<spark::SparkMax>(*m_bus, kRightMotorCanID);
    }
    catch (const std::exception& e)
    {
        m_right.reset();
        m_left.reset();
        m_bus.reset();
        g_myRobotNode->writeLog("BalanceDrive: CAN init failed, motors stay disabled");
        g_myRobotNode->writeLog("%s", e.what());
        return -1;
    }

    // Wake the bus so the SPARKs start sending status (they stay silent until they see traffic).
    m_bus->sendWakeFrame();
    g_myRobotNode->writeLog("BalanceDrive: CAN open on can0, motors disabled until robot is enabled");
    return 0;
}

bool BalanceDrive::isEnabled()
{
    return m_isEnabled;
}

void BalanceDrive::setEnable(bool isEnabled)
{
    if (m_isEnabled == isEnabled)
    {
        return; // No change
    }

    m_isEnabled = isEnabled;
    m_dutyL = m_dutyR = 0.0;
    g_myRobotNode->writeLog(isEnabled ? "BalanceDrive: enabled" : "BalanceDrive: disabled, motors zeroed");
}

bool BalanceDrive::motorsAlive() const
{
    return m_left && m_right && m_left->isAlive() && m_right->isAlive();
}

// addLeftControl and addRightControl are joystick offsets in range -1.0 to 1.0
// They are added on top of the balance output AFTER clamping, so they can
// temporarily override balance — keep them small during tuning.
void BalanceDrive::update(double addLeftControl, double addRightControl)
{
    m_updatedThisCycle = true;

    if (!m_isEnabled)
    {
        m_dutyL = m_dutyR = 0.0;
        return;
    }

    // --- Tilt error ---
    // accel.x increases when deck tilts forward (toward horizontal)
    // accel.x ≈ kBalancePoint when deck is at the ~45° balance angle
    // Positive error = tilted forward = need to drive forward to catch up
    double tilt_error = g_myRobotNode->m_imu_linear_acceleration[RobotNode::IMU_AXES::X] - kBalancePoint;

    // --- Proportional correction, clamped ---
    double balance_output = std::clamp(kP * tilt_error, -kMaxOutput, kMaxOutput);

    // No fresh IMU data: never balance on a stale or default (zero) reading.
    // Joystick offsets still apply, so the wheels can be driven by hand.
    if (!g_myRobotNode->isImuFresh(0.1))
    {
        balance_output = 0.0;
        g_myRobotNode->writeLog("BalanceDrive: no fresh IMU data, balance term held at 0");
    }

    // --- Motor outputs ---
    // Right motor: positive = clockwise from right = forward = corrects positive tilt
    // Left motor:  inverted because motors face opposite directions
    // Joystick offsets are already sign-corrected in Robot::balancingPeriodic()
    m_dutyR = std::clamp(balance_output + addRightControl, -kDutyLimit, kDutyLimit);
    m_dutyL = std::clamp(-balance_output + addLeftControl, -kDutyLimit, kDutyLimit);

    // Values change every cycle, so repeat-suppression can't help: limit to 10 lines/s.
    static auto nextLog = std::chrono::steady_clock::now();
    if (std::chrono::steady_clock::now() >= nextLog)
    {
        nextLog = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        g_myRobotNode->writeLog("BalanceDrive: tilt_err=%.3f bal=%.3f L=%.3f R=%.3f",
                                tilt_error, balance_output, m_dutyL, m_dutyR);
    }
}

void BalanceDrive::periodic()
{
    if (!m_bus)
    {
        return;   // CAN failed to open: no heartbeat is ever sent, so the SPARKs stay disabled
    }

    m_bus->poll();

    // States that are enabled but don't drive the wheels yet (airborne, landing, hunkered)
    // never call update(); they get duty 0.
    if (!m_updatedThisCycle)
    {
        m_dutyL = m_dutyR = 0.0;
    }
    m_updatedThisCycle = false;

    const bool alive = motorsAlive();
    const bool drive = m_isEnabled && alive;
    if (m_isEnabled && !alive)
    {
        g_myRobotNode->writeLog("BalanceDrive: SPARK MAX status missing, motors held disabled");
    }

    const uint64_t enableBits =
        spark::CanBus::deviceBit(kLeftMotorCanID) | spark::CanBus::deviceBit(kRightMotorCanID);
    m_bus->sendHeartbeat(drive ? enableBits : 0);
    m_left->setDutyCycle(drive ? static_cast<float>(m_dutyL) : 0.0f);
    m_right->setDutyCycle(drive ? static_cast<float>(m_dutyR) : 0.0f);
}
