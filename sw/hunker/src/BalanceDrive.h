#ifndef BALANCEDRIVE_H
#define BALANCEDRIVE_H

#include <cmath>
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

    // Tilt error from the balance point (rad, + = tipped forward) and its rate (rad/s, same sign).
    // Returns false if the IMU data is invalid or older than 100 ms.
    bool getTiltError(double& error, double& rate) const;

    // Balancing may only START within this much of the balance point (Y press).
    static constexpr double kStartWindow = 5.0 * M_PI / 180.0;    // rad
    // While balancing, a larger error than this stops balancing (duty 0, back to LANDING).
    static constexpr double kTiltCutoff = 20.0 * M_PI / 180.0;    // rad

private:
    bool m_isEnabled = false;
    bool m_updatedThisCycle = false;
    double m_balanceOutput = 0.0;   // last balance term, for the log

    void logTilt();   // 10 lines/s, in every state
    double m_dutyL = 0.0;
    double m_dutyR = 0.0;

    // Declaration order matters: the motors must be destroyed before the bus.
    std::unique_ptr<spark::CanBus> m_bus;
    std::unique_ptr<spark::SparkMax> m_left;
    std::unique_ptr<spark::SparkMax> m_right;

    static constexpr int kLeftMotorCanID = 1;
    static constexpr int kRightMotorCanID = 2;

    // Tilt, from the BNO085's fused orientation (see imuTilt() in BalanceDrive.cpp).
    // tilt = asin(u.x), where u is world "up" expressed in the IMU frame.
    // Hand-tilt test, 2026-11-18 (battery off): resting on the kickstand tilt = 89.4°;
    // held at the balance point tilt = 36.6° (two holds, 35.6–37.3°); tipping FORWARD makes
    // tilt DECREASE, and the gyro tilt rate has the same sign as d(tilt)/dt.
    // (The old 0.779 rad = 44.6° came from accel.x, which reads 1.36× too high.)
    static constexpr double kBalanceAngle = 0.639;   // rad (36.6°)

    // Proportional gain, duty per radian of tilt error (error + = tipped forward).
    // Holding a tilt against gravity needs ≈ 0.37 duty/rad (9.6 kg, 5:1, 8" wheel,
    // NEO-class stall torque); 0.8 ≈ 2× that. 5° error → 0.07 duty.
    static constexpr double kP = 0.8;

    // Derivative gain, duty per rad/s of tilt rate (from the gyro, not differenced).
    // Same sign as kP (rate sign verified by the hand-tilt test). 0 until the wheel
    // direction is checked; then about 0.05.
    static constexpr double kD = 0.0;

    // Maximum motor output magnitude during balance (0.0 to 1.0)
    // Keep small until PD tuning is complete
    static constexpr double kMaxOutput = 0.15;

    // Hard limit on what is actually sent to each SPARK, balance + joystick combined.
    // Bench-test value: raise only after motor type and encoder are confirmed in REV Hardware Client 2.
    static constexpr double kDutyLimit = 0.10;
};

#endif   // BALANCEDRIVE_H
