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

    static void setPower(Motor::MOTOR_TYPE typ, double power); // -1.0 to 1.0
    static double getPower(Motor::MOTOR_TYPE typ);             // -1.0 to 1.0
    static bool isReverse(Motor::MOTOR_TYPE typ);
    static int getTurnCount(Motor::MOTOR_TYPE typ); // 12 pulses per revolution, divide by gear ratio to get angle of leg extension

    static const int m_gpio_for_turn_count[Motor::NUMBER_OF_MOTORS];
    static const int m_gpio_for_pwm[Motor::NUMBER_OF_MOTORS];
    static const int m_gpio_for_direction[Motor::NUMBER_OF_MOTORS];

    static double m_power[Motor::NUMBER_OF_MOTORS];
    static sig_atomic_t m_turn_count[Motor::NUMBER_OF_MOTORS];
};

#endif // MOTOR_H
