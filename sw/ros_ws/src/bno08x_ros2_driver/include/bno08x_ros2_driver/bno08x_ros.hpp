#ifndef BNO08x_ROS_H
#define BNO08x_ROS_H

#include <rclcpp/rclcpp.hpp>                    
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "bno08x_ros2_driver/bno08x.hpp"

#include <chrono>
#include <thread>
#include <atomic>
#include <gpiod.h>

class BNO08x_ROS : public rclcpp::Node 
{
public:                                          
    BNO08x_ROS();
    ~BNO08x_ROS();

private:
    // Initialization
    void init_parameters();                      
    void init_comms();
    void init_sensor();
    bool enable_reports();      // rotation vector, accelerometer, calibrated gyro
    bool recover_sensor();      // soft reset + re-open + re-enable reports; interrupt thread only
    void init_gpio();
    void cleanup_gpio();

    // Callbacks
    void sensor_callback(void *cookie, sh2_SensorValue_t *sensor_value);
     void interrupt_thread_func();

    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr          imu_publisher_;

    sensor_msgs::msg::Imu           imu_msg_;
    sensor_msgs::msg::MagneticField mag_msg_;

    std::string frame_id_;
    bool        publish_imu_{false};
    bool        publish_magnetic_field_{false};
    int         imu_rate_{100};
    int         magnetic_field_rate_{100};
    uint8_t     imu_received_flag_{0};

    // Stall watchdog (interrupt thread only). A sensor that resets or locks up (e.g. a
    // jostled wire) forgets its report configuration and goes silent; recover it.
    std::chrono::steady_clock::time_point last_data_time_{};
    int recovery_count_{0};

    // Hardware driver
    BNO08x          *bno08x_{nullptr};
    CommInterface   *comm_interface_{nullptr};

    // GPIO / interrupt
    std::thread           interrupt_thread_;
    std::atomic<bool>     running_{false};
    struct gpiod_chip    *gpio_chip_{nullptr};
    struct gpiod_line    *int_line_{nullptr};
    std::string           gpio_chip_name_;
    int                   gpio_line_num_{27};
};

#endif   // BNO08x_ROS_H