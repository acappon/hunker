#ifndef MY_GPIO_H
#define MY_GPIO_H

class MyGpio
{
public:
    typedef enum
    {
        // LED GPIOs are chosen to be close to the end of the connector, to keep
        // wires
        // shorter
        GPIO_LED_ENABLE = 17,
        GPIO_LED_FAULT = 4,

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
