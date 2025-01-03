

class Motor
{
public:
     typedef enum 
    {
        GPIO_L_WHEEL_DIR=0,
        GPIO_L_WHEEL_PWM=0,
        GPIO_L_WHEEL_COUNT=0,

        GPIO_R_WHEEL_DIR=0,
        GPIO_R_WHEEL_PWM=0,
        GPIO_R_WHEEL_COUNT=0,

        GPIO_L_KNEE_DIR=0,
        GPIO_L_KNEE_PWM=0,
        GPIO_L_KNEE_COUNT=0,

        GPIO_R_KNEE_DIR=0,
        GPIO_R_KNEE_PWM=0,
        GPIO_R_KNEE_COUNT=0,
    } GPIO_PIN;

   Motor() {}
   Motor(Motor::GPIO_PIN gpio_for_direction, Motor::GPIO_PIN gpio_for_pwm, Motor::GPIO_PIN gpio_for_turn_count);
   ~Motor();

    void setPower(double power, bool reverse); // -1.0 to 1.0
    int getTurnCount();          // 12 pulses per revolution, divide by gear ratio to get angle of leg extension

private:
    int m_gpio_for_direction;
    int m_gpio_for_pwm;
    int m_gpio_for_turn_count;

    int lgpio_chip; 

    sig_atomic_t turn_count;
};