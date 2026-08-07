#pragma once
// Stub for lgpio library used in unit tests

#define LG_SET_PULL_NONE 0
#define LG_GPIO_BUSY -1

#ifdef __cplusplus
extern "C" {
#endif

int  lgGpiochipOpen(int chipnum);
int  lgGpiochipClose(int handle);
int  lgGpioClaimOutput(int handle, int flags, int gpio, int level);
int  lgGpioWrite(int handle, int gpio, int level);
int  lgGpioRead(int handle, int gpio);
const char *lguErrorText(int error);

#ifdef __cplusplus
}
#endif
