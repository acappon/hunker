#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
public:
    typedef enum
    {
        RWheel,
        LWheel,
        RKnee,
        LKnee,
        NUMBER_OF_MOTORS
    } MOTOR_TYPE;

    Motor();
    ~Motor();

    static int init();

    static std::string motorName(Motor::MOTOR_TYPE typ);

    static bool isEnabled(Motor::MOTOR_TYPE typ);
    static bool isForward(Motor::MOTOR_TYPE typ);
    static void setPower(Motor::MOTOR_TYPE typ, double power); // -1.0 to 1.0
    static double getPower(Motor::MOTOR_TYPE typ);             // -1.0 to 1.0
    static int getTurnCount(Motor::MOTOR_TYPE typ);            // 12 pulses per revolution, divide by gear ratio to get angle of leg extension

    static const MyGpio::GPIO_PIN m_gpio_for_enable[Motor::NUMBER_OF_MOTORS];
    static const MyGpio::GPIO_PIN m_gpio_for_direction[Motor::NUMBER_OF_MOTORS];
    static const MyGpio::GPIO_PIN m_gpio_for_pwm[Motor::NUMBER_OF_MOTORS];
    static const MyGpio::GPIO_PIN m_gpio_for_turn_count[Motor::NUMBER_OF_MOTORS];

    static double m_power[Motor::NUMBER_OF_MOTORS];
    static int m_turn_count[Motor::NUMBER_OF_MOTORS];
};

#endif // MOTOR_H
