#ifndef Joystick_H
#define Joystick_H
#include <stdint.h>
#include <stdbool.h>

bool Init_Joystick(void);
uint32_t Read_X_Joystick(void);
uint32_t Read_Y_Joystick(void);
#endif