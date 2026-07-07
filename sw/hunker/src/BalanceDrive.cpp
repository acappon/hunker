#include "common.h"

#include <SparkMax.hpp>

extern "C"
{
#include <lgpio.h>
}

extern std::shared_ptr<RobotNode> g_myRobotNode;

static const int kLeftMotorCanID = 1;
static const int kRightMotorCanID = 2;

static const int kP = 0.00005;
static const int kI = 0.0;
static const int kD = 0.0;
static const int kF = 0.00025;

static SparkMax leftMotor{"can0", kLeftMotorCanID};
static SparkMax rightMotor{"can0", kRightMotorCanID};

BalanceDrive::BalanceDrive()
{
}

BalanceDrive::~BalanceDrive()
{
}

int BalanceDrive::init()
{
    leftMotor.SetMotorType(MotorType::kBrushless);
    leftMotor.SetIdleMode(IdleMode::kCoast);
    leftMotor.SetP(0, kP);
    leftMotor.SetI(0, kI);
    leftMotor.SetD(0, kD);
    leftMotor.SetF(0, kF);
    leftMotor.BurnFlash();

    rightMotor.SetMotorType(MotorType::kBrushless);
    rightMotor.SetIdleMode(IdleMode::kCoast);
    rightMotor.SetP(0, kP);
    rightMotor.SetI(0, kI);
    rightMotor.SetD(0, kD);
    rightMotor.SetF(0, kF);
    rightMotor.BurnFlash();

    g_myRobotNode->writeLog("BalanceDrive: motors initialized in coast mode");
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

    if (!isEnabled)
    {
        // Explicitly command zero when disabling so motors don't coast
        // from the last nonzero command
        leftMotor.SetDutyCycle(0.0);
        rightMotor.SetDutyCycle(0.0);
        g_myRobotNode->writeLog("BalanceDrive: disabled, motors zeroed");
    }
    else
    {
        g_myRobotNode->writeLog("BalanceDrive: enabled");
    }
}


// addLeftControl and addRightControl are joystick offsets in range -1.0 to 1.0
// They are added on top of the balance output AFTER clamping, so they can
// temporarily override balance — keep them small during tuning.
void BalanceDrive::update(double addLeftControl, double addRightControl)
{
    leftMotor.Heartbeat();
    rightMotor.Heartbeat();

    if (!m_isEnabled)
    {
        leftMotor.SetDutyCycle(0.0);
        rightMotor.SetDutyCycle(0.0);
        return;
    }

    // --- Tilt error ---
    // accel.x increases when deck tilts forward (toward horizontal)
    // accel.x ≈ kBalancePoint when deck is at the ~45° balance angle
    // Positive error = tilted forward = need to drive forward to catch up
    double tilt_error = g_myRobotNode->m_imu_linear_acceleration[RobotNode::IMU_AXES::X] - kBalancePoint;

    // --- Proportional correction ---
    double balance_output = kP * tilt_error;

    // Clamp balance output
    if (balance_output > kMaxOutput)
        balance_output = kMaxOutput;
    if (balance_output < -kMaxOutput)
        balance_output = -kMaxOutput;

    // --- Motor outputs ---
    // Right motor: positive = clockwise from right = forward = corrects positive tilt
    // Left motor:  inverted because motors face opposite directions
    // Joystick offsets are already sign-corrected in Robot::balancingPeriodic()
    double power_R = balance_output + addRightControl;
    double power_L = -balance_output + addLeftControl;

    // Final clamp to [-1, 1]
    if (power_R > 1.0)
        power_R = 1.0;
    if (power_R < -1.0)
        power_R = -1.0;
    if (power_L > 1.0)
        power_L = 1.0;
    if (power_L < -1.0)
        power_L = -1.0;

    g_myRobotNode->writeLog("BalanceDrive: tilt_err=%.3f bal=%.3f L=%.3f R=%.3f",
                            tilt_error, balance_output, power_L, power_R);

    leftMotor.SetDutyCycle(power_L);
    rightMotor.SetDutyCycle(power_R);
}
