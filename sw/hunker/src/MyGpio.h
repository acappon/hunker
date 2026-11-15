#ifndef MY_GPIO_H
#define MY_GPIO_H

class MyGpio
{
public:
    typedef enum
    {
        // Orange Pi 5, line numbers on gpiochip1 (GPIO bank 1, see MyGpio.cpp).
        // Header pin 16 = GPIO1_D3 = line 27; pin 18 = GPIO1_D2 = line 26; GND on pin 14.
        // Do NOT use header pins 8/10/11/13 (GPIO4_A3/A4/B2/B3): they double as DDR PHY
        // debug pins, and LEDs on 11/13 stopped the board booting (2026-11-14).
        GPIO_LED_ENABLE = 27,   // pin 16, drives the lamp transistor's base
        GPIO_LED_FAULT = 26,    // pin 18, LED assembly with built-in resistor

    } GPIO_PIN;

    MyGpio();
    ~MyGpio();
    bool initEnableAndFaultLED();

    bool isValidPin(MyGpio::GPIO_PIN pin);
    bool gpioWrite(MyGpio::GPIO_PIN pin, bool value);
    bool gpioRead(MyGpio::GPIO_PIN pin);

    void setEnableLED(bool state);
    void setFaultLED(bool state);

    void enableWheelMotors(bool isEnabled);

    bool setMotorPower(MyGpio::GPIO_PIN dirPin, MyGpio::GPIO_PIN pwmPin,
                       bool isFwd, double power);

private:
    int m_lgpio_chip;
};

#endif // MY_GPIO_H
