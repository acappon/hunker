#ifndef UTILS_H
#define UTILS_H

#include <rclcpp/rclcpp.hpp>

// Clamp a value to [min_val, max_val].
inline double clampDouble(double value, double min_val, double max_val)
{
    if (value > max_val)
        return max_val;
    if (value < min_val)
        return min_val;
    return value;
}

// Create a QoS profile suitable for high-rate sensor subscriptions
// (best-effort, volatile durability, keep-last 10).
inline rclcpp::QoS createSensorQoS()
{
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.keep_last(10)
        .best_effort()
        .durability_volatile();
    return qos;
}

// Macro for the recurring exception-handling pattern:
//   log a context message, log e.what(), log stack trace, set FAULT_EXCEPTION.
// Requires `this` to be a RobotNode* (or accessed via g_myRobotNode).
#define HANDLE_EXCEPTION(node_ptr, context, exception)           \
    do {                                                         \
        (node_ptr)->writeLog(context);                           \
        (node_ptr)->writeLog((exception).what());                \
        (node_ptr)->writeLog("Stack trace: %s",                  \
                             (node_ptr)->getStackTrace().c_str()); \
        (node_ptr)->m_faultIndicator.setFault(                   \
            FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);  \
    } while (0)

#endif // UTILS_H
