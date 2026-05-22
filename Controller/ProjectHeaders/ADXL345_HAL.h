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

/**
 * @brief Initialize ADXL345 using SPI1.
 *
 * @return true if ADXL345 is detected successfully.
 * @return false if DEVID is not 0xE5.
 */
bool ADXL345_Init(void);

/**
 * @brief Read ADXL345 device ID.
 *
 * @return Device ID. Should be 0xE5.
 */
uint8_t ADXL345_ReadID(void);

/**
 * @brief Read raw X/Y/Z acceleration data.
 *
 * @param data Pointer to ADXL345_RawData_t.
 */
void ADXL345_ReadRaw(ADXL345_RawData_t *data);

/**
 * @brief Read X/Y/Z acceleration data in g.
 *
 * @param data Pointer to ADXL345_GData_t.
 */
void ADXL345_ReadG(ADXL345_GData_t *data);

/**
 * @brief Write one ADXL345 register.
 *
 * @param reg Register address.
 * @param value Value to write.
 */
void ADXL345_WriteReg(uint8_t reg, uint8_t value);

/**
 * @brief Read one ADXL345 register.
 *
 * @param reg Register address.
 * @return Register value.
 */
uint8_t ADXL345_ReadReg(uint8_t reg);

/**
 * @brief Read multiple ADXL345 registers.
 *
 * @param startReg Starting register address.
 * @param buffer Data buffer.
 * @param length Number of bytes to read.
 */
void ADXL345_ReadMulti(uint8_t startReg, uint8_t *buffer, uint8_t length);

#endif