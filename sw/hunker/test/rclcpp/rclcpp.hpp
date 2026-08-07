#pragma once
// Minimal rclcpp stub for unit testing without ROS2

#include <string>
#include <memory>
#include <functional>
#include <chrono>
#include <cstdarg>

namespace rclcpp
{

class Duration
{
public:
    Duration(int sec, int nsec) : sec_(sec), nsec_(nsec) {}
    double seconds() const { return sec_ + nsec_ / 1e9; }
private:
    int sec_;
    int nsec_;
};

class Time
{
public:
    Time() : ns_(0) {}
    Time(int64_t ns) : ns_(ns) {}
    int64_t nanoseconds() const { return ns_; }
    double seconds() const { return ns_ / 1e9; }
    Duration operator-(const Time &other) const
    {
        return Duration(0, static_cast<int>(ns_ - other.ns_));
    }

private:
    int64_t ns_;
};

struct QoSInitialization
{
    static QoSInitialization from_rmw(int) { return {}; }
};

class QoS
{
public:
    QoS(QoSInitialization) {}
    QoS &keep_last(int) { return *this; }
    QoS &best_effort() { return *this; }
    QoS &durability_volatile() { return *this; }
};

class Logger
{
public:
    const char *get_name() const { return "test"; }
};

class TimerBase
{
public:
    using SharedPtr = std::shared_ptr<TimerBase>;
    void reset() {}
};

// Generic subscription / publisher stubs
template <typename MsgT>
class Subscription
{
public:
    using SharedPtr = std::shared_ptr<Subscription<MsgT>>;
    void reset() {}
};

template <typename MsgT>
class Publisher
{
public:
    using SharedPtr = std::shared_ptr<Publisher<MsgT>>;
    void reset() {}
    void publish(const MsgT &) {}
};

inline int rmw_qos_profile_sensor_data = 0;

class Node
{
public:
    Node(const std::string &) {}
    virtual ~Node() = default;

    Logger get_logger() { return Logger(); }
    Time now() { return Time(0); }

    template <typename MsgT>
    typename Subscription<MsgT>::SharedPtr create_subscription(
        const std::string &, const QoS &,
        std::function<void(const typename MsgT::SharedPtr)>)
    {
        return std::make_shared<Subscription<MsgT>>();
    }

    template <typename MsgT>
    typename Publisher<MsgT>::SharedPtr create_publisher(const std::string &, int)
    {
        return std::make_shared<Publisher<MsgT>>();
    }

    TimerBase::SharedPtr create_wall_timer(
        std::chrono::milliseconds,
        std::function<void()>)
    {
        return std::make_shared<TimerBase>();
    }
};

} // namespace rclcpp

// Stub logging macros
#define RCLCPP_INFO(logger, ...) (void)0
#define RCLCPP_ERROR(logger, ...) (void)0
