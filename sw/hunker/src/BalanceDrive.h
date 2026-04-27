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
};




#endif   // BALANCEDRIVE_H