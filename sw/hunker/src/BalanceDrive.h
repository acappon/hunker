#ifndef BALANCEDRIVE_H
#define BALANCEDRIVE_H

class BalanceDrive
{
public:
    BalanceDrive();
    ~BalanceDrive();

    int init();

    bool isEnabled();
    void setEnable(bool isEnabled);

    void update(double addLeftControl, double addRightControl);   // Balancing control loop is in SparkMAx controller, joystick input tilts the desired "UP" vector for balancing

private:
    bool m_isEnabled = false;

    // Tilt signal at balance point, from linear_acceleration.x
    // Measured empirically: accel.x ≈ 6.89 when deck is at ~45° balance angle
    static constexpr double kBalancePoint = 6.89;

    // Proportional gain — start VERY small.  A value of 0.01 means a 1 m/s²
    // tilt error produces 1% motor power.  Tune upward carefully.
    static constexpr double kP = 0.01;

    // Maximum motor output magnitude during balance (0.0 to 1.0)
    // Keep small until PD tuning is complete
    static constexpr double kMaxOutput = 0.15;
};

#endif   // BALANCEDRIVE_H
