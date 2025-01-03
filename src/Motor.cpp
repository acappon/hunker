
#include <csignal>
#include <memory>

extern "C"
{
#include <lgpio.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "IMU_bno055.h"
#include "Motor.h"
#include "RobotNode.h"

extern std::shared_ptr<RobotNode> g_myRobot;

volatile int turn_count = 0;

void turnCountISR(int num_alerts, lgGpioAlert_p alerts, void *user)
{
    (void)alerts; // Avoid unused parameter warning
    (void)user; // Avoid unused parameter warning

    for (int i = 0; i < num_alerts; ++i) {
        //if (alerts[i].level == LG_GPIO_CHANGED_TO_HIGH) 
        {
            turn_count++;
        }
    }
}

Motor::Motor(Motor::GPIO_PIN gpio_for_direction, Motor::GPIO_PIN gpio_for_pwm, Motor::GPIO_PIN gpio_for_turn_count)
    : m_gpio_for_direction(gpio_for_direction), m_gpio_for_pwm(gpio_for_pwm), m_gpio_for_turn_count(gpio_for_turn_count)
{
    // Prepare interrupt service for turn count GPIO pin
        // Open the GPIO chip
   lgpio_chip = lgGpiochipOpen(0);
    if (lgpio_chip < 0) {
        std::string msg = "Failed to open GPIO chip: " + std::to_string(lgpio_chip); 
      //  g_myRobot->writeLog(msg);
        return;
    }

    if (lgGpioClaimOutput(lgpio_chip, 0, m_gpio_for_direction, 0) < 0 ||
        lgGpioClaimOutput(lgpio_chip, 0, m_gpio_for_pwm, 0) < 0 ||
        lgGpioClaimInput(lgpio_chip, 0, m_gpio_for_turn_count) < 0) {
      //  g_myRobot->writeLog("Failed to claim GPIO lines");
        return;
    }

    if (lgGpioSetAlertsFunc(lgpio_chip, m_gpio_for_turn_count, turnCountISR, nullptr) < 0) {
       // g_myRobot->writeLog("Failed to set alert function");
        return;
    }

}

Motor::~Motor() 
{
    lgGpiochipClose(lgpio_chip);
}

void Motor::setPower(double power, bool reverse)
{
    // Set direction
    lgGpioWrite(lgpio_chip, m_gpio_for_direction, reverse ? 1 : 0);

     // Convert power to PWM value (assuming power is between -1 and 1)
    int pwm_value = static_cast<int>((power + 1) * 512); // Scale to 0-1024 range
    lgGpioWrite(lgpio_chip, m_gpio_for_pwm, pwm_value);
}

int Motor::getTurnCount()
{
    return turn_count;
}

