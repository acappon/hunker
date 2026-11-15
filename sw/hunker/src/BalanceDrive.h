#ifndef BALANCEDRIVE_H
#define BALANCEDRIVE_H

#include <memory>

#include "SparkMaxCan.h"

class BalanceDrive
{
public:
    BalanceDrive();
    ~BalanceDrive();   // destroys motors then bus; the bus sends disable heartbeats if it ever enabled

    int init();   // opens can0; returns -1 (and leaves the motors disabled) on failure

    bool isEnabled();
    void setEnable(bool isEnabled);

    // Computes the wheel duty cycles for this cycle. Sends nothing; periodic() does.
    void update(double addLeftControl, double addRightControl);   // joystick offsets, -1..1

    // Call exactly once per robot cycle, after the state handlers. Reads CAN status, then
    // sends the heartbeat and both duty cycles. Missing this for ~100 ms disables the SPARKs.
    void periodic();

    // True when both SPARK MAX controllers have reported status within the last 100 ms.
    bool motorsAlive() const;

private:
    bool m_isEnabled = false;
    bool m_updatedThisCycle = false;
    double m_dutyL = 0.0;
    double m_dutyR = 0.0;

    // Declaration order matters: the motors must be destroyed before the bus.
    std::unique_ptr<spark::CanBus> m_bus;
    std::unique_ptr<spark::SparkMax> m_left;
    std::unique_ptr<spark::SparkMax> m_right;

    static constexpr int kLeftMotorCanID = 1;
    static constexpr int kRightMotorCanID = 2;

    // Tilt, from the BNO085's fused orientation (see imuTilt() in BalanceDrive.cpp).
    // tilt = asin(u.x), where u is world "up" expressed in the IMU frame. This is the same
    // axis and sign as the old raw signal (accel.x ≈ 9.81·u.x at rest) but without the
    // error from the robot's own acceleration.
    // Balance point: old empirical accel.x ≈ 6.89 at the ~45° deck angle → asin(6.89/9.81).
    static constexpr double kBalanceAngle = 0.779;   // rad (44.6°)

    // Proportional gain, duty per radian of tilt error. 0.07 matches the old
    // 0.01 per m/s² near the balance point (1 rad ≈ 7 m/s² of accel.x there).
    // ≈ 0.0012 duty per degree. Tune upward carefully.
    static constexpr double kP = 0.07;

    // Derivative gain, duty per rad/s of tilt rate (from the gyro, not differenced).
    // Starts at 0: P-only, as before. Add after the tilt-rate sign is checked by hand.
    static constexpr double kD = 0.0;

    // Maximum motor output magnitude during balance (0.0 to 1.0)
    // Keep small until PD tuning is complete
    static constexpr double kMaxOutput = 0.15;

    // Hard limit on what is actually sent to each SPARK, balance + joystick combined.
    // Bench-test value: raise only after motor type and encoder are confirmed in REV Hardware Client 2.
    static constexpr double kDutyLimit = 0.10;
};

#endif   // BALANCEDRIVE_H
