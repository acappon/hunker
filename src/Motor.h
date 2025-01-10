

class Motor
/**
 * @file Motor.h
 * @brief Header file for the Motor class, which controls the motor's direction, PWM, and turn count.
 *
 * This file contains the definition of the Motor class, which provides methods to control the motor's power,
 * direction, and to keep track of the turn count using GPIO pins.
 *
 * GPIO pins are defined based on the output from `sudo gpioinfo` for GPIO chip #4.
 *
 * @enum GPIO_PIN
 * @brief Enumeration for GPIO pins used for motor control.
 *
 * @var GPIO_PIN::GPIO_L_WHEEL_DIR
 * GPIO pin for left wheel direction.
 * @var GPIO_PIN::GPIO_L_WHEEL_PWM
 * GPIO pin for left wheel PWM.
 * @var GPIO_PIN::GPIO_L_WHEEL_COUNT
 * GPIO pin for left wheel turn count.
 * @var GPIO_PIN::GPIO_R_WHEEL_DIR
 * GPIO pin for right wheel direction.
 * @var GPIO_PIN::GPIO_R_WHEEL_PWM
 * GPIO pin for right wheel PWM.
 * @var GPIO_PIN::GPIO_R_WHEEL_COUNT
 * GPIO pin for right wheel turn count.
 * @var GPIO_PIN::GPIO_L_KNEE_DIR
 * GPIO pin for left knee direction.
 * @var GPIO_PIN::GPIO_L_KNEE_PWM
 * GPIO pin for left knee PWM.
 * @var GPIO_PIN::GPIO_L_KNEE_COUNT
 * GPIO pin for left knee turn count.
 * @var GPIO_PIN::GPIO_R_KNEE_DIR
 * GPIO pin for right knee direction.
 * @var GPIO_PIN::GPIO_R_KNEE_PWM
 * GPIO pin for right knee PWM.
 * @var GPIO_PIN::GPIO_R_KNEE_COUNT
 * GPIO pin for right knee turn count.
 *
 * @enum TURN_COUNT_TYPE
 * @brief Enumeration for types of turn counters.
 *
 * @var TURN_COUNT_TYPE::RWheel
 * Turn counter for the right wheel.
 * @var TURN_COUNT_TYPE::LWheel
 * Turn counter for the left wheel.
 * @var TURN_COUNT_TYPE::RKnee
 * Turn counter for the right knee.
 * @var TURN_COUNT_TYPE::LKnee
 * Turn counter for the left knee.
 * @var TURN_COUNT_TYPE::NUMBER_OF_TURN_COUNTERS
 * Total number of turn counters.
 *
 * @class Motor
 * @brief Class to control motor direction, PWM, and turn count.
 *
 * The Motor class provides methods to set motor power, initialize ISRs for turn counting, and get the turn count.
 *
 * @fn Motor::Motor()
 * Default constructor for the Motor class.
 *
 * @fn Motor::Motor(Motor::GPIO_PIN gpio_for_direction, Motor::GPIO_PIN gpio_for_pwm)
 * Constructor for the Motor class.
 * @param gpio_for_direction GPIO pin for motor direction.
 * @param gpio_for_pwm GPIO pin for motor PWM.
 *
 * @fn Motor::~Motor()
 * Destructor for the Motor class.
 *
 * @fn static int Motor::initIsrsForTurnCount(Motor::GPIO_PIN gpio_for_turn_count_RW, Motor::GPIO_PIN gpio_for_turn_count_LW, Motor::GPIO_PIN gpio_for_turn_count_RK, Motor::GPIO_PIN gpio_for_turn_count_LK)
 * Initialize ISRs for turn counting.
 * @param gpio_for_turn_count_RW GPIO pin for right wheel turn count.
 * @param gpio_for_turn_count_LW GPIO pin for left wheel turn count.
 * @param gpio_for_turn_count_RK GPIO pin for right knee turn count.
 * @param gpio_for_turn_count_LK GPIO pin for left knee turn count.
 * @return int Status of the initialization.
 *
 * @fn void Motor::setPower(double power, bool reverse)
 * Set the power for the motor.
 * @param power Power level from -1.0 to 1.0.
 * @param reverse Boolean to indicate if the direction should be reversed.
 *
 * @fn int Motor::getTurnCount(TURN_COUNT_TYPE typ)
 * Get the turn count for a specific type.
 * @param typ Type of turn counter.
 * @return int Turn count.
 *
 * @var int Motor::m_gpio_for_direction
 * GPIO pin for motor direction.
 *
 * @var int Motor::m_gpio_for_pwm
 * GPIO pin for motor PWM.
 *
 * @var static int Motor::m_lgpio_chip
 * GPIO chip number.
 *
 * @var static int Motor::m_gpio_for_turn_count[NUMBER_OF_TURN_COUNTERS]
 * Array of GPIO pins for turn counting.
 *
 * @var static sig_atomic_t Motor::m_turn_count[NUMBER_OF_TURN_COUNTERS]
 * Array of turn counts.
 */
{
public:
    /*  All GPIO are on GPIO chip #4
        Output from sudo gpioinfo:
    gpiochip4 - 54 lines:
            line   0:     "ID_SDA"       unused   input  active-high
            line   1:     "ID_SCL"       unused   input  active-high
            line   2:      "GPIO2"       unused   input  active-high
            line   3:      "GPIO3"       unused   input  active-high
            line   4:      "GPIO4"       unused   input  active-high
            line   5:      "GPIO5"       unused   input  active-high
            line   6:      "GPIO6"       unused   input  active-high
            line   7:      "GPIO7"   "spi0 CS1"  output   active-low [used]
            line   8:      "GPIO8"   "spi0 CS0"  output   active-low [used]
            line   9:      "GPIO9"       unused   input  active-high
            line  10:     "GPIO10"       unused   input  active-high
            line  11:     "GPIO11"       unused   input  active-high
            line  12:     "GPIO12"       unused   input  active-high
            line  13:     "GPIO13"       unused   input  active-high
            line  14:     "GPIO14"       unused   input  active-high
            line  15:     "GPIO15"       unused   input  active-high
            line  16:     "GPIO16"       unused   input  active-high
            line  17:     "GPIO17"       unused   input  active-high
            line  18:     "GPIO18"       unused   input  active-high
            line  19:     "GPIO19"       unused   input  active-high
            line  20:     "GPIO20"       unused   input  active-high
            line  21:     "GPIO21"       unused   input  active-high
            line  22:     "GPIO22"       unused   input  active-high
            line  23:     "GPIO23"       unused   input  active-high
            line  24:     "GPIO24"       unused   input  active-high
            line  25:     "GPIO25"       unused   input  active-high
            line  26:     "GPIO26"       unused   input  active-high
            line  27:     "GPIO27"       unused   input  active-high
            line  30:   "HOST_SDA"       unused   input  active-high
            line  31:   "HOST_SCL"       unused   input  active-high
     */
    typedef enum
    {
        RWheel,
        LWheel,
        RKnee,
        LKnee,
        NUMBER_OF_MOTORS
    } MOTOR_TYPE;

    typedef enum
    {
        GPIO_R_WHEEL_DIR = 2, // 0 and 1 are used for I2C
        GPIO_R_WHEEL_PWM = 3,
        GPIO_R_WHEEL_COUNT = 4,

        GPIO_L_WHEEL_DIR = 5,
        GPIO_L_WHEEL_PWM = 6,
        GPIO_L_WHEEL_COUNT = 9, // 7&8 are used for SPI

        GPIO_R_KNEE_DIR = 10,
        GPIO_R_KNEE_PWM = 11,
        GPIO_R_KNEE_COUNT = 12,

        GPIO_L_KNEE_DIR = 13,
        GPIO_L_KNEE_PWM = 14,
        GPIO_L_KNEE_COUNT = 15,
    } GPIO_PIN;

    Motor();
    ~Motor();

    static int init();

    static void setPower(Motor::MOTOR_TYPE typ, double power); // -1.0 to 1.0
    static double getPower(Motor::MOTOR_TYPE typ);                // -1.0 to 1.0
    static bool isReverse(Motor::MOTOR_TYPE typ);
    static int getTurnCount(Motor::MOTOR_TYPE typ); // 12 pulses per revolution, divide by gear ratio to get angle of leg extension

    static int m_lgpio_chip;
    static const int m_gpio_for_turn_count[Motor::NUMBER_OF_MOTORS];
    static const int m_gpio_for_pwm[Motor::NUMBER_OF_MOTORS];
    static const int m_gpio_for_direction[Motor::NUMBER_OF_MOTORS];

    static double m_power[Motor::NUMBER_OF_MOTORS];
    static sig_atomic_t m_turn_count[Motor::NUMBER_OF_MOTORS];
};
