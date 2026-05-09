#include "bno08x_ros2_driver/bno08x_ros.hpp"
#include "bno08x_ros2_driver/i2c_interface.hpp"
#include "bno08x_ros2_driver/uart_interface.hpp"

constexpr uint8_t ROTATION_VECTOR_RECEIVED = 0x01;
constexpr uint8_t ACCELEROMETER_RECEIVED   = 0x02;
constexpr uint8_t GYROSCOPE_RECEIVED       = 0x04;

BNO08x_ROS::BNO08x_ROS()
    : Node("bno08x_ros")
{  
    this->init_parameters();
    this->init_comms();
    this->init_sensor();
    init_gpio();

    if (publish_imu_) {
        this->imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        RCLCPP_INFO(this->get_logger(), "IMU Publisher created");
        RCLCPP_INFO(this->get_logger(), "IMU Rate: %d", imu_rate_);
    }

    RCLCPP_INFO(this->get_logger(), "BNO08X ROS Node started.");
}

BNO08x_ROS::~BNO08x_ROS() {
    delete bno08x_;
    delete comm_interface_;
}

/**
 * @brief Initialize the communication interface
 * 
 * communication interface based on the parameters
 */
void BNO08x_ROS::init_comms() {
    bool i2c_enabled, uart_enabled;
    this->get_parameter("i2c.enabled", i2c_enabled);
    this->get_parameter("uart.enabled", uart_enabled);

    if (i2c_enabled) {
        std::string device;
        std::string address;
        this->get_parameter("i2c.bus", device);
        this->get_parameter("i2c.address", address);
        RCLCPP_INFO(this->get_logger(), "Communication Interface: I2C");
        try {
            comm_interface_ = new I2CInterface(device, std::stoi(address, nullptr, 16));
        } catch (const std::bad_alloc& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "Failed to allocate memory for I2CInterface object: %s", e.what());
            throw std::runtime_error("I2CInterface object allocation failed");
        }
    } else if (uart_enabled) {
        RCLCPP_INFO(this->get_logger(), "Communication Interface: UART");
        std::string device;
        this->get_parameter("uart.device", device);
        try{
            comm_interface_ = new UARTInterface(device);
        } catch (const std::bad_alloc& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "Failed to allocate memory for UARTInterface object: %s", e.what());
            throw std::runtime_error("UARTInterface object allocation failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "No communication interface enabled!");
        throw std::runtime_error("Communication interface setup failed");
    }
}

/**
 * @brief Initialize the parameters
 * 
 * This function initializes the parameters for the node
 * 
 */
void BNO08x_ROS::init_parameters() {
    this->declare_parameter<std::string>("frame_id", "bno085");

    this->declare_parameter<bool>("publish.imu.enabled", true);
    this->declare_parameter<int>("publish.imu.rate", 100);

    this->declare_parameter<bool>("i2c.enabled", true);
    this->declare_parameter<std::string>("i2c.bus", "/dev/i2c-7");
    this->declare_parameter<std::string>("i2c.address", "0x4A");
    this->declare_parameter<bool>("uart.enabled", false);
    this->declare_parameter<std::string>("uart.device", "/dev/ttyACM0");

    this->declare_parameter<std::string>("interrupt.gpio_chip", "gpiochip0");
    this->declare_parameter<int>        ("interrupt.gpio_line", 24);

    this->get_parameter("frame_id", frame_id_);

    this->get_parameter("publish.imu.enabled", publish_imu_);
    this->get_parameter("publish.imu.rate", imu_rate_);

    this->get_parameter("interrupt.gpio_chip", gpio_chip_name_);
    this->get_parameter("interrupt.gpio_line", gpio_line_num_);
}

void BNO08x_ROS::init_gpio() {
    gpio_chip_ = gpiod_chip_open_by_name(gpio_chip_name_.c_str());
    if (!gpio_chip_) {
        throw std::runtime_error("Failed to open GPIO chip: " + gpio_chip_name_);
    }

    int_line_ = gpiod_chip_get_line(gpio_chip_, gpio_line_num_);
    if (!int_line_) {
        gpiod_chip_close(gpio_chip_);
        throw std::runtime_error("Failed to get GPIO line: " 
                                  + std::to_string(gpio_line_num_));
    }

    // Active-low interrupt: watch for falling edge
    // "bno085_int" is just a consumer label for debugging (shows in gpioinfo)
    int ret = gpiod_line_request_falling_edge_events(int_line_, "bno085_int");
    if (ret < 0) {
        gpiod_chip_close(gpio_chip_);
        throw std::runtime_error("Failed to request GPIO edge events");
    }

    RCLCPP_INFO(this->get_logger(), "GPIO interrupt configured: %s line %d",
                gpio_chip_name_.c_str(), gpio_line_num_);
}

/**
 * @brief Initialize the sensor
 * 
 * This function initializes the sensor and enables the required sensor reports
 * 
 */
void BNO08x_ROS::init_sensor() {

    try {
        bno08x_ = new BNO08x(comm_interface_, std::bind(&BNO08x_ROS::sensor_callback, this, 
                                        std::placeholders::_1, std::placeholders::_2), this);
    } catch (const std::bad_alloc& e) {
        RCLCPP_ERROR(this->get_logger(), 
                        "Failed to allocate memory for BNO08x object: %s", e.what());
        throw std::runtime_error("BNO08x object allocation failed");
    }

    if (!bno08x_->begin()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO08X sensor");
        throw std::runtime_error("BNO08x initialization failed");
    }

    if(!this->bno08x_->enable_report(SH2_ROTATION_VECTOR, 
                                        1000000/this->imu_rate_)){              // Hz to us
      RCLCPP_ERROR(this->get_logger(), "Failed to enable rotation vector sensor");
    }
    if(!this->bno08x_->enable_report(SH2_ACCELEROMETER,
                                        1000000/this->imu_rate_)){              // Hz to us
        RCLCPP_ERROR(this->get_logger(), "Failed to enable accelerometer sensor");
    }
    if(!this->bno08x_->enable_report(SH2_GYROSCOPE_CALIBRATED, 
                                        1000000/this->imu_rate_)){              // Hz to us
        RCLCPP_ERROR(this->get_logger(), "Failed to enable gyroscope sensor");
    }
}   

/**
 * @brief Callback function for sensor events
 * 
 * @param cookie Pointer to the object that called the function, not used here
 * @param sensor_value The sensor value from parsing the sensor event buffer
 * 
 */
void BNO08x_ROS::sensor_callback(void *cookie, sh2_SensorValue_t *sensor_value) {
	DEBUG_LOG("Sensor Callback");
	switch(sensor_value->sensorId){
		case SH2_ROTATION_VECTOR:
			this->imu_msg_.orientation.x = sensor_value->un.rotationVector.i;
			this->imu_msg_.orientation.y = sensor_value->un.rotationVector.j;
			this->imu_msg_.orientation.z = sensor_value->un.rotationVector.k;
			this->imu_msg_.orientation.w = sensor_value->un.rotationVector.real;
			imu_received_flag_ |= ROTATION_VECTOR_RECEIVED;
			break;
		case SH2_ACCELEROMETER:
			this->imu_msg_.linear_acceleration.x = sensor_value->un.accelerometer.x;
			this->imu_msg_.linear_acceleration.y = sensor_value->un.accelerometer.y;
			this->imu_msg_.linear_acceleration.z = sensor_value->un.accelerometer.z;
			imu_received_flag_ |= ACCELEROMETER_RECEIVED;
			break;
		case SH2_GYROSCOPE_CALIBRATED:
			this->imu_msg_.angular_velocity.x = sensor_value->un.gyroscope.x;
			this->imu_msg_.angular_velocity.y = sensor_value->un.gyroscope.y;
			this->imu_msg_.angular_velocity.z = sensor_value->un.gyroscope.z;
			imu_received_flag_ |= GYROSCOPE_RECEIVED;
			break;
		default:
			break;
	}

	if(imu_received_flag_ == (ROTATION_VECTOR_RECEIVED | ACCELEROMETER_RECEIVED | GYROSCOPE_RECEIVED)){
		this->imu_msg_.header.frame_id = this->frame_id_;
		this->imu_msg_.header.stamp.sec = this->get_clock()->now().seconds();
		this->imu_msg_.header.stamp.nanosec = this->get_clock()->now().nanoseconds();
		this->imu_publisher_->publish(this->imu_msg_);
		imu_received_flag_ = 0;
	}

}

// ── interrupt_thread_func ────────────────────────────────────────────────────
// This replaces poll_timer_callback entirely.
// Runs in its own thread, blocks on GPIO edge, then services the sensor.

void BNO08x_ROS::interrupt_thread_func() {
    RCLCPP_INFO(this->get_logger(), "Interrupt thread started");

    // Timeout so we can check running_ flag and exit cleanly on shutdown
    constexpr timespec timeout = {0, 50'000'000};   // 50ms watchdog

    while (running_.load()) {
        // Block here until falling edge or timeout
        int ret = gpiod_line_event_wait(int_line_, &timeout);

        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "GPIO event wait error: %d", ret);
            break;
        }

        if (ret == 0) {
            // Timeout — no interrupt in 50ms.
            // At 400Hz we expect one every 2.5ms, so 50ms means sensor stalled.
            // Log a warning but keep running; sensor may recover.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 
                                 1000,   // throttle to once/sec
                                 "BNO085: no interrupt in 50ms — sensor stalled?");
            continue;
        }

        // ret == 1: edge detected — consume the event from the kernel buffer
        gpiod_line_event event;
        if (gpiod_line_event_read(int_line_, &event) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to read GPIO event");
            continue;
        }

        // Service the sensor — this calls sensor_callback() synchronously
        // We're in a regular thread so ROS2 publisher calls are safe
        this->bno08x_->poll();
    }

    RCLCPP_INFO(this->get_logger(), "Interrupt thread exiting");
}

