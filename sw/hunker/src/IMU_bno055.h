#ifndef IMU_BNO055_H
#define IMU_BNO055_H

#define BNO055_ADDRESS_A 0x28
#define BNO055_ID 0xA0

class IMU_bno055
{
public:
public:
    typedef struct
    {
        // "up" vector is opposite direction of gravity
        // basic accelerometer output, no need to integrate to get position
        // If magnitude is very low, robot is airborne/ballistic, (is_airborne == true)
        double up_pitch;
        double up_roll;
        double up_yaw;
        double up_magnitude;
        bool is_airborne;

        // "fall" vector is derivative of gyro rate
        // Not used when moving toward "up"
        // Balancing PID will counter "falling" away from "up"
        double fall_pitch;
        double fall_roll;
        double fall_yaw;
        double fall_magnitude;

        double w_orientation;
        double x_orientation;
        double y_orientation;
        double z_orientation;

        double w_angular_velocity;
        double x_angular_velocity;
        double y_angular_velocity;
        double z_angular_velocity;

        double w_linear_acceleration;
        double x_linear_acceleration;
        double y_linear_acceleration;
        double z_linear_acceleration;
    } IMU_DATA;

    typedef enum
    {
        I2C_STATUS_OK,
        I2C_NOT_INITIALIZED,
        I2C_OPEN_FAILED,
        I2C_BUS_ACCESS_FAILED,
        I2C_BNO055_NOT_DETECTED,
        I2C_WRITE_FAILED,
        I2C_READ_FAILED
    } I2C_STATUS;

    IMU_bno055();
    void update();

    IMU_DATA pos;

private:
    unsigned char read8(unsigned char reg);
    signed short read16(unsigned char reg);
    void write8(unsigned char reg, unsigned char value);

    int i2c_file_;
    I2C_STATUS i2c_status;
};

#endif // IMU_BNO055_H
