#ifndef ROBOT_H
#define ROBOT_H

class Robot
{
public:
    typedef enum
    {
        ROBOT_STATE_DISABLED,
        ROBOT_STATE_BALANCING,
        ROBOT_STATE_LANDING,
        ROBOT_STATE_HUNKERED,
        ROBOT_STATE_AIRBORNE,
        NUMBER_OF_ROBOT_STATES
    } ROBOT_STATE;

    Robot();
    void init();
    ~Robot();

    void robotFunction();

    int getTurnCount();

    void periodic();
    void disabledPeriodic();
    void balancingPeriodic();
    void airbornePeriodic();
    void landingPeriodic();
    void hunkeredPeriodic();

    public:
    IMU_bno055 imu;
    Motor m_myMotors;

private:
    ROBOT_STATE m_robotState;
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