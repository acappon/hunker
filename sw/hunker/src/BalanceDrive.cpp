#include "common.h"

#include <SparkMax.h>
#include <config/SparkMaxConfig.h>

extern "C"
{
#include <lgpio.h>
}

extern std::shared_ptr<RobotNode> g_myRobotNode;

static const int kLeftMotorCanID = 1;
static const int kRightMotorCanID = 2;

static rev::spark::SparkMax leftMotor{kLeftMotorCanID, rev::spark::SparkMax::MotorType::kBrushless};
static rev::spark::SparkMax rightMotor{kRightMotorCanID, rev::spark::SparkMax::MotorType::kBrushless};

BalanceDrive::BalanceDrive()
{
}

BalanceDrive::~BalanceDrive()
{
}

int BalanceDrive::init()
{
    // Configure both motors for coast mode so the robot can be pushed to
    // balance position by hand without fighting the controllers.
    // Switch to brake mode once balance tuning is complete.
    rev::spark::SparkMaxConfig config;
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast);

    leftMotor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kNoPersistParameters);

    rightMotor.Configure(
        config,
        rev::ResetMode::kResetSafeParameters,
        rev::PersistMode::kNoPersistParameters);

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
        leftMotor.Set(0.0);
        rightMotor.Set(0.0);
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
    if (!m_isEnabled)
    {
        leftMotor.Set(0.0);
        rightMotor.Set(0.0);
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

    leftMotor.Set(power_L);
    rightMotor.Set(power_R);
}
