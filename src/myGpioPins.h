#ifndef MY_GPIO_PINS_H
#define MY_GPIO_PINS_H

typedef enum
{
    // LED GPIOs are chosen to be close to the end of the connector, to keep
    // wires
    // shorter
    GPIO_LED_ENABLE = 4,
    GPIO_LED_FAULT = 17,

    // GPIOs are chosen for each wheel to be close to each other on the
    // connector
    GPIO_R_WHEEL_DIR = 22,
    GPIO_R_WHEEL_PWM = 23,
    GPIO_R_WHEEL_COUNT = 27,

    GPIO_L_WHEEL_DIR = 6,
    GPIO_L_WHEEL_PWM = 6,
    GPIO_L_WHEEL_COUNT = 9,

    GPIO_R_KNEE_DIR = 5,
    GPIO_R_KNEE_PWM = 6,
    GPIO_R_KNEE_COUNT = 12,

    GPIO_L_KNEE_DIR = 13,
    GPIO_L_KNEE_PWM = 16,
    GPIO_L_KNEE_COUNT = 26,
 
    GPIO_TEST = 24,
} GPIO_PIN;

#endif  // MY_GPIO_PINS_H
