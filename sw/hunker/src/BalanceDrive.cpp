#include "common.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

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
    // REV Hardware Client 2. No parameters are written here, and flash is never burned.
    // (Status 2 is switched on in periodic(): that is a telemetry request, not a parameter.)
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

bool BalanceDrive::getTiltError(double& error, double& rate) const
{
    double tilt = 0.0;
    if (!imuTilt(tilt, rate) || !g_myRobotNode->isImuFresh(0.1))
    {
        error = rate = 0.0;
        return false;
    }
    // tilt DECREASES when the robot tips forward (hand-tilt test, 2026-11-18), so the
    // error and rate are negated: + = tipped forward = need to drive forward to catch up.
    error = kBalanceAngle - tilt;
    rate = -rate;
    return true;
}

// addLeftControl and addRightControl are joystick offsets in range -1.0 to 1.0, as raw
// driver intent (+ = that side forward). Motor inversion (right motor) is applied here.
void BalanceDrive::update(double addLeftControl, double addRightControl)
{
    m_updatedThisCycle = true;

    if (!m_isEnabled)
    {
        m_dutyL = m_dutyR = 0.0;
        m_balanceOutput = 0.0;
        return;
    }

    double tilt_error = 0.0, tilt_rate = 0.0;
    const bool tiltOk = getTiltError(tilt_error, tilt_rate);

    // --- PD correction, clamped ---
    double balance_output = std::clamp(kP * tilt_error + kD * tilt_rate, -kMaxOutput, kMaxOutput);

    // No fresh, valid IMU data, or tilted past the cutoff: never balance on it.
    // Robot::periodic() leaves BALANCING in both cases; this is a second line of defence.
    if (!tiltOk)
    {
        balance_output = 0.0;
        g_myRobotNode->writeLog("BalanceDrive: no fresh IMU data, balance term held at 0");
    }
    else if (std::fabs(tilt_error) > kTiltCutoff)
    {
        balance_output = 0.0;
    }
    m_balanceOutput = balance_output;

    // --- Motor outputs ---
    // Right motor: NEGATIVE duty = wheel forward (gantry test 2026-11-25: R=-0.043 drove
    //              the right wheel forward). So right is inverted.
    // Left motor:  mirrored on the other leg, so positive = forward (confirmed in the
    //              repeat test once the left belt was loosened).
    m_dutyR = std::clamp(-(balance_output + addRightControl), -kDutyLimit, kDutyLimit);
    m_dutyL = std::clamp(balance_output + addLeftControl, -kDutyLimit, kDutyLimit);
}

// Wheel ground speed from the NEO hall encoders (Status 2), m/s, + = that wheel forward.
// Encoder signs (hand-turn test 2026-11-25): wheel forward = left +, right −, the same as duty.
bool BalanceDrive::getWheelSpeeds(double& left, double& right) const
{
    left = right = 0.0;
    if (!m_left || !m_right)
    {
        return false;
    }
    const long ageL = m_left->status2AgeMs(), ageR = m_right->status2AgeMs();
    const float rpmL = m_left->status2().velocityRpm, rpmR = m_right->status2().velocityRpm;
    if (ageL < 0 || ageL > 100 || ageR < 0 || ageR > 100 || !std::isfinite(rpmL) || !std::isfinite(rpmR))
    {
        return false;
    }
    left = rpmL * kMotorRpmToWheelMps;
    right = -rpmR * kMotorRpmToWheelMps;
    return true;
}

void BalanceDrive::logTilt()
{
    // Values change every cycle, so repeat-suppression can't help: limit to 10 lines/s.
    static auto nextLog = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (now < nextLog)
    {
        return;
    }
    nextLog = now + std::chrono::milliseconds(100);

    double tilt = 0.0, tilt_rate = 0.0;
    if (!imuTilt(tilt, tilt_rate) || !g_myRobotNode->isImuFresh(0.1))
    {
        g_myRobotNode->writeLog("BalanceDrive: tilt unknown (no fresh IMU data)");
        return;
    }
    // tilt and accel-tilt should agree when the robot is still; a sign or axis mismatch
    // shows up here first.
    const double accelTilt = std::asin(std::clamp(
        g_myRobotNode->m_imu_linear_acceleration[RobotNode::IMU_AXES::X] / 9.80665, -1.0, 1.0));
    double vL = 0.0, vR = 0.0;
    const bool speedOk = getWheelSpeeds(vL, vR);
    char speeds[64];
    if (speedOk)
        std::snprintf(speeds, sizeof speeds, "vL=%+.2f vR=%+.2f m/s", vL, vR);
    else
        std::snprintf(speeds, sizeof speeds, "wheel speed unknown");
    g_myRobotNode->writeLog("BalanceDrive: tilt=%.1f° rate=%.1f°/s accelTilt=%.1f° err=%.1f° bal=%.3f L=%.3f R=%.3f %s",
                            tilt * kRadToDeg, tilt_rate * kRadToDeg, accelTilt * kRadToDeg,
                            (kBalanceAngle - tilt) * kRadToDeg, m_balanceOutput, m_dutyL, m_dutyR, speeds);
}

void BalanceDrive::periodic()
{
    if (!m_bus)
    {
        return;   // CAN failed to open: no heartbeat is ever sent, so the SPARKs stay disabled
    }

    m_bus->poll();

    // Status 2 (encoder velocity/position) is off after every SPARK power-up. Ask for it
    // whenever it is missing, at most once a second (also covers a SPARK brownout/reboot).
    const auto now = std::chrono::steady_clock::now();
    if (now >= m_nextStatus2Request)
    {
        m_nextStatus2Request = now + std::chrono::seconds(1);
        for (auto* m : {m_left.get(), m_right.get()})
        {
            const long age = m->status2AgeMs();
            if (age < 0 || age > 200)
            {
                m->enableEncoderStatus();
            }
        }
    }

    // States that are enabled but don't drive the wheels yet (airborne, landing, hunkered)
    // never call update(); they get duty 0.
    if (!m_updatedThisCycle)
    {
        m_dutyL = m_dutyR = 0.0;
        m_balanceOutput = 0.0;
    }
    m_updatedThisCycle = false;

    logTilt();

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
