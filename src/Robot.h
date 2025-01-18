#ifndef ROBOT_H
#define ROBOT_H

class Robot
{
public:
    Robot();
    void init();
    ~Robot();

    void robotFunction();

    void setPower(double power, bool reverse);
    int getTurnCount();

    void periodic();
    void disabledPeriodic();
    void balancingPeriodic();
    void airbornePeriodic();
    void landingPeriodic();

public:
    IMU_bno055 imu;
    Motor *pMotors;

private:
    double m_deck_pitch;
    double m_deck_roll;

    // radians
    // knee motor turn counts indicate leg extension.
    // The deck would be in this orientation if balanced vertically
    void estimateDeckOrientation();

    bool isBalancingPossible(); // true if the robot is not airborne, and the deck vertical is not too far from "up"
    bool isFullyFolded();       // true if the robot is in a compact, folded state, (lower limit switch is active)
};

#endif // ROBOT_H