#ifndef PAIRING_HAL_H
#define PAIRING_HAL_H
#include <stdint.h>
#include <stdbool.h>

bool SevenSeg_Init(void);

void SevenSeg_DisplayDigit(uint8_t digit);
void SevenSeg_DisplayRaw(uint8_t pattern);
void SevenSeg_DisplayOff(void);
void SevenSeg_DisplayDP(bool on);

#endif