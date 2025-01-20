#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "IMU_bno055.h"
#include "sensor_msgs/msg/imu.hpp"

IMU_bno055::IMU_bno055()
{
    i2c_status = I2C_STATUS_OK;

    // Open I2C bus
    const char *i2c_device = "/dev/i2c-1";
    if ((i2c_file_ = open(i2c_device, O_RDWR)) < 0)
    {
        i2c_status = I2C_OPEN_FAILED;
        return;
    }

    // Set I2C address
    if (ioctl(i2c_file_, I2C_SLAVE, BNO055_ADDRESS_A) < 0)
    {
        i2c_status = I2C_BUS_ACCESS_FAILED;
        return;
    }

    // Check BNO055 ID
    if (read8(0x00) != BNO055_ID)
    {
        i2c_status = I2C_BNO055_NOT_DETECTED;
        return;
    }

    // Initialize BNO055
    write8(0x3D, 0x00); // Set to config mode
    usleep(10000);
    write8(0x3F, 0x20); // Use external crystal
    usleep(10000);
    write8(0x3D, 0x0C); // Set to NDOF mode
    usleep(10000);
}

void IMU_bno055::update()
{
    pos.x_orientation = read16(0x1A) / 16384.0;
    pos.y_orientation = read16(0x1C) / 16384.0;
    pos.z_orientation = read16(0x1E) / 16384.0;
    pos.w_orientation = read16(0x20) / 16384.0;

    pos.x_angular_velocity = read16(0x14) / 900.0;
    pos.z_angular_velocity = read16(0x16) / 900.0;
    pos.x_angular_velocity = read16(0x18) / 900.0;

    pos.x_linear_acceleration = read16(0x08) / 100.0;
    pos.z_linear_acceleration = read16(0x0A) / 100.0;
    pos.x_linear_acceleration = read16(0x0C) / 100.0;
}

unsigned char IMU_bno055::read8(unsigned char reg)
{
    unsigned char value;
    if (write(i2c_file_, &reg, 1) != 1)
    {
        i2c_status = I2C_WRITE_FAILED;
    }
    if (read(i2c_file_, &value, 1) != 1)
    {
        i2c_status = I2C_READ_FAILED;
    }
    return value;
}

int16_t IMU_bno055::read16(unsigned char reg)
{
    unsigned char buffer[2];
    if (write(i2c_file_, &reg, 1) != 1)
    {
        i2c_status = I2C_WRITE_FAILED;
    }
    if (read(i2c_file_, buffer, 2) != 2)
    {
        i2c_status = I2C_READ_FAILED;
    }
    return (buffer[1] << 8) | buffer[0];
}

void IMU_bno055::write8(unsigned char reg, unsigned char value)
{
    unsigned char buffer[2] = {reg, value};
    if (write(i2c_file_, buffer, 2) != 2)
    {
        i2c_status = I2C_WRITE_FAILED;
    }
}
