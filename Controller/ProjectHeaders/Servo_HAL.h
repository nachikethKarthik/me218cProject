#ifndef SERVO_HAL_H
#define SERVO_HAL_H

#include <stdint.h>
#include <stdbool.h>

void Servo_Init();
void Servo_SetAngle(uint8_t angle);
void Servo_SetPulseWidth(uint16_t pw);;
#endif