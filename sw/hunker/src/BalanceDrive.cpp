#include "common.h"

#include <algorithm>
#include <cmath>

static constexpr double kRadToDeg = 180.0 / M_PI;

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

// Tilt angle and tilt rate from the IMU's fused orientation and calibrated gyro.
// u = world "up" expressed in the sensor frame, from the rotation-vector quaternion
// (sensor→world): the third row of its rotation matrix. At rest the accelerometer reads
// ≈ 9.81·u, so tilt = asin(u.x) has the same sense as the old accel.x signal.
// For a world-fixed vector seen from the body, du/dt = u × ω, so
// d/dt asin(u.x) = (u.y·ω.z − u.z·ω.y) / sqrt(1 − u.x²).
// Returns false if the quaternion is not usable (e.g. all zeros before the first message).
static bool imuTilt(double& tilt, double& tiltRate)
{
    using A = RobotNode::IMU_AXES;
    const auto& q = g_myRobotNode->m_imu_orientation;
    const auto& w = g_myRobotNode->m_imu_angular_velocity;

    double qw = q[A::W], qx = q[A::X], qy = q[A::Y], qz = q[A::Z];
    const double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (!(n > 0.5))   // also catches NaN
    {
        return false;
    }
    qw /= n; qx /= n; qy /= n; qz /= n;

    const double ux = 2.0 * (qx * qz - qw * qy);
    const double uy = 2.0 * (qy * qz + qw * qx);
    const double uz = 1.0 - 2.0 * (qx * qx + qy * qy);

    tilt = std::asin(std::clamp(ux, -1.0, 1.0));
    tiltRate = (uy * w[A::Z] - uz * w[A::Y]) / std::max(std::sqrt(1.0 - ux * ux), 1e-3);
    return true;
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
    // tilt increases when the deck tilts forward (toward horizontal), as accel.x did.
    // tilt ≈ kBalanceAngle at the ~45° balance angle.
    // Positive error = tilted forward = need to drive forward to catch up
    double tilt = 0.0, tilt_rate = 0.0;
    const bool tiltOk = imuTilt(tilt, tilt_rate);
    const double tilt_error = tilt - kBalanceAngle;

    // --- PD correction, clamped ---
    double balance_output = std::clamp(kP * tilt_error + kD * tilt_rate, -kMaxOutput, kMaxOutput);

    // No fresh, valid IMU data: never balance on a stale or default (zero) reading.
    // Joystick offsets still apply, so the wheels can be driven by hand.
    if (!tiltOk || !g_myRobotNode->isImuFresh(0.1))
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
        // tilt and accel-tilt should agree when the robot is still; a sign or axis mismatch
        // shows up here first.
        const double accelTilt = std::asin(std::clamp(
            g_myRobotNode->m_imu_linear_acceleration[RobotNode::IMU_AXES::X] / 9.80665, -1.0, 1.0));
        g_myRobotNode->writeLog("BalanceDrive: tilt=%.1f° rate=%.1f°/s accelTilt=%.1f° err=%.1f° bal=%.3f L=%.3f R=%.3f",
                                tilt * kRadToDeg, tilt_rate * kRadToDeg, accelTilt * kRadToDeg,
                                tilt_error * kRadToDeg, balance_output, m_dutyL, m_dutyR);
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
