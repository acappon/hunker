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

    typedef enum
    {
        ROBOT_FRAME,    // Tank drive.  Fwd/Back stick means fwd/back relative to robot.  Right/Left means % differential power per wheel
        FIELD_FRAME,    // Requires IMU.  POwer on robot while standing behind it with controller pointed same direction.
                        // Direction stick is pushed is direction robot will turn to, magnitude of push = speed
        NUMBER_OF_DRIVE_TYPES
    } DRIVE_TYPE;

    Robot();
    void init();
    ~Robot();

    void robotFunction();

    std::string getRobotStateText();
    int getTurnCount();

    void periodic();
    void disabledPeriodic();
    void balancingPeriodic();
    void airbornePeriodic();
    void landingPeriodic();
    void hunkeredPeriodic();

public:
    BNO080 m_imu;
    Motor m_myMotors;

private:
    DRIVE_TYPE m_driveType;
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