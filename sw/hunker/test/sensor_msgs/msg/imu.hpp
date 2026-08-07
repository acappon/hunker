#pragma once
#include <memory>

namespace sensor_msgs { namespace msg {

struct Imu
{
    using SharedPtr = std::shared_ptr<Imu>;
    struct { double w, x, y, z; } orientation;
    struct { double x, y, z; } linear_acceleration;
    struct { double x, y, z; } angular_velocity;
};

}} // namespace sensor_msgs::msg
