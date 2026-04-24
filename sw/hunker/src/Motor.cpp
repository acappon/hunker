
#include "common.h"

extern "C"
{
#include <lgpio.h>
#include <SparkMax.h>
#include <config/SparkMaxConfig.h>
}

extern std::shared_ptr<RobotNode> g_myRobotNode;

double Motor::m_power[Motor::NUMBER_OF_MOTORS] = {0.0};

Motor::Motor()
{
}

Motor::~Motor()
{
}

int Motor::init()
{
    rev::spark::SparkMax neoMotor{1, rev::spark::SparkMax::MotorType::kBrushless};

    return 0;
}

std::string Motor::motorName(Motor::MOTOR_TYPE typ)
{
    switch (typ)
    {
    case RWheel:
        return "RWheel";
    case LWheel:
        return "LWheel";
    case RKnee:
        return "RKnee";
    case LKnee:
        return "LKnee";
    default:
        return "Unknown";
    }
}

void Motor::setPower(Motor::MOTOR_TYPE typ, double power) // -1.0 to 1.0
{
    m_power[typ] = power;

    // set motor power
}

bool Motor::isEnabled(Motor::MOTOR_TYPE typ)
{
    return m_power[typ] != 0.0;
}

bool Motor::isForward(Motor::MOTOR_TYPE typ)
{
    return m_power[typ] > 0.0;
}

double Motor::getPower(Motor::MOTOR_TYPE typ)
{
    return m_power[typ];
}
