#ifndef ADXL345_HAL_H
#define ADXL345_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ADXL345_RawData_t;

typedef struct {
    float x_g;
    float y_g;
    float z_g;
} ADXL345_GData_t;

bool ADXL345_Init(void);
uint8_t ADXL345_ReadID(void);
void ADXL345_ReadRaw(ADXL345_RawData_t *data);
void ADXL345_ReadG(ADXL345_GData_t *data);
void ADXL345_WriteReg(uint8_t reg, uint8_t value);
uint8_t ADXL345_ReadReg(uint8_t reg);
void ADXL345_ReadMulti(uint8_t startReg, uint8_t *buffer, uint8_t length);
uint8_t ADXL345_TestReadID(void);
#endif