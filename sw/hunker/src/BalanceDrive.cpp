
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
static rev::spark::SparkMax rightMotor{kLeftMotorCanID, rev::spark::SparkMax::MotorType::kBrushless};


BalanceDrive::BalanceDrive()
{
}

BalanceDrive::~BalanceDrive()
{

}


int  BalanceDrive::init()
{
    return -1;
}


bool  BalanceDrive::isEnabled()
{
    return false;

}

void  BalanceDrive::setEnable(bool isEnabled)
{

}

// Balancing control loop is in SparkMAx controller, joystick input tilts the desired "UP" vector for balancing
void  BalanceDrive::update(double addLeftControl, double addRightControl)
{

}
    
