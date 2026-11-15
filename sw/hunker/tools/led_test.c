// Blink the Hunker status lights to check wiring, without running the robot code.
//   enable lamp: header pin 16 = GPIO1_D3 = gpiochip1 line 27 (drives a transistor base)
//   fault LED:   header pin 18 = GPIO1_D2 = gpiochip1 line 26
// Build: gcc -O2 -Wall -o led_test led_test.c -lgpiod     Run: ./led_test
// Needs group gpio (see sysconfig/etc/udev/rules.d/60-gpio.rules). Leaves both lines low.
#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>

static int blink(struct gpiod_line* line, const char* name, int times)
{
    printf("%s: blinking %d times\n", name, times);
    fflush(stdout);
    for (int i = 0; i < times; ++i)
    {
        if (gpiod_line_set_value(line, 1) < 0 || usleep(500000) < 0 ||
            gpiod_line_set_value(line, 0) < 0)
            return -1;
        usleep(500000);
    }
    return 0;
}

int main(void)
{
    struct gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip1");
    if (!chip)
    {
        perror("open gpiochip1");
        return 1;
    }
    struct gpiod_line* enable = gpiod_chip_get_line(chip, 27);
    struct gpiod_line* fault = gpiod_chip_get_line(chip, 26);
    if (!enable || !fault || gpiod_line_request_output(enable, "hunker_led_test", 0) < 0 ||
        gpiod_line_request_output(fault, "hunker_led_test", 0) < 0)
    {
        perror("request lines 26/27 as outputs");
        gpiod_chip_close(chip);
        return 1;
    }

    int rc = 0;
    rc |= blink(enable, "ENABLE lamp (pin 16)", 3);
    sleep(1);
    rc |= blink(fault, "FAULT LED  (pin 18)", 3);

    gpiod_line_set_value(enable, 0);
    gpiod_line_set_value(fault, 0);
    gpiod_line_release(enable);
    gpiod_line_release(fault);
    gpiod_chip_close(chip);
    puts(rc ? "error while blinking" : "done, both off");
    return rc ? 1 : 0;
}
