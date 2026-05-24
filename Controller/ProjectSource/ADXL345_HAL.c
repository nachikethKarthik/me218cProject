#include "ADXL345_HAL.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// ================================================================
// ADXL345 Register Addresses
// ================================================================

#define ADXL345_REG_DEVID        0x00
#define ADXL345_REG_BW_RATE      0x2C
#define ADXL345_REG_POWER_CTL    0x2D
#define ADXL345_REG_DATA_FORMAT  0x31
#define ADXL345_REG_DATAX0       0x32

#define ADXL345_DEVID_VALUE      0xE5

// ================================================================
// ADXL345 SPI Command Bits
// ================================================================

#define ADXL345_READ_BIT         0x80
#define ADXL345_MULTI_BYTE_BIT   0x40

// ================================================================
// ADXL345 Scale Factor
// ================================================================
//
// In FULL_RES mode, ADXL345 scale factor is approximately 3.9 mg/LSB.
// So acceleration in g = raw_value * 0.0039
//

#define ADXL345_SCALE_FACTOR_G   0.0039f

// ================================================================
// Pin Definitions
// ================================================================
//
// Your wiring:
//
// RA3  -> ADXL345 CS
// RB5  -> ADXL345 SDI  = PIC32 SDO1
// RB8  -> ADXL345 SDO  = PIC32 SDI1
// RB14 -> ADXL345 SCLK = PIC32 SCK1
//

#define ADXL345_CS_LAT           LATAbits.LATA3
#define ADXL345_CS_TRIS          TRISAbits.TRISA3

// ================================================================
// Private Function Prototypes
// ================================================================

static void ADXL345_SPI1_Init(void);
static uint8_t ADXL345_SPI1_Transfer(uint8_t data);
static void ADXL345_CS_Low(void);
static void ADXL345_CS_High(void);

// ================================================================
// Public Functions
// ================================================================

bool ADXL345_Init(void)
{
    uint8_t id;

    ADXL345_SPI1_Init();

    id = ADXL345_ReadID();

    if (id != ADXL345_DEVID_VALUE) {
        return false;
    }

    ADXL345_WriteReg(ADXL345_REG_BW_RATE, 0x0A);
    ADXL345_WriteReg(ADXL345_REG_DATA_FORMAT, 0x08);
    ADXL345_WriteReg(ADXL345_REG_POWER_CTL, 0x08);

    return true;
}

uint8_t ADXL345_ReadID(void)
{
    return ADXL345_ReadReg(ADXL345_REG_DEVID);
}

void ADXL345_ReadRaw(ADXL345_RawData_t *data)
{
    uint8_t buffer[6];

    if (data == 0) {
        return;
    }

    ADXL345_ReadMulti(ADXL345_REG_DATAX0, buffer, 6);

    /*
     * ADXL345 data format is little-endian:
     *
     * X = DATAX1:DATAX0
     * Y = DATAY1:DATAY0
     * Z = DATAZ1:DATAZ0
     */
    data->x = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)(((uint16_t)buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)(((uint16_t)buffer[5] << 8) | buffer[4]);
}

void ADXL345_ReadG(ADXL345_GData_t *data)
{
    ADXL345_RawData_t raw;

    if (data == 0) {
        return;
    }

    ADXL345_ReadRaw(&raw);

    data->x_g = (float)raw.x * ADXL345_SCALE_FACTOR_G;
    data->y_g = (float)raw.y * ADXL345_SCALE_FACTOR_G;
    data->z_g = (float)raw.z * ADXL345_SCALE_FACTOR_G;
}

void ADXL345_WriteReg(uint8_t reg, uint8_t value)
{
    ADXL345_CS_Low();

    /*
     * Write command:
     * bit7 = 0
     * bit6 = 0 for single byte
     */
    ADXL345_SPI1_Transfer(reg & 0x3F);
    ADXL345_SPI1_Transfer(value);

    ADXL345_CS_High();
}

uint8_t ADXL345_ReadReg(uint8_t reg)
{
    uint8_t value;

    ADXL345_CS_Low();

    /*
     * Read command:
     * bit7 = 1
     * bit6 = 0 for single byte
     */
    ADXL345_SPI1_Transfer(ADXL345_READ_BIT | (reg & 0x3F));
    value = ADXL345_SPI1_Transfer(0x00);

    ADXL345_CS_High();

    return value;
}

void ADXL345_ReadMulti(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
    uint8_t i;

    if (buffer == 0) {
        return;
    }

    ADXL345_CS_Low();

    /*
     * Multi-byte read command:
     * bit7 = 1
     * bit6 = 1
     */
    ADXL345_SPI1_Transfer(
        ADXL345_READ_BIT |
        ADXL345_MULTI_BYTE_BIT |
        (startReg & 0x3F)
    );

    for (i = 0; i < length; i++) {
        buffer[i] = ADXL345_SPI1_Transfer(0x00);
    }

    ADXL345_CS_High();
}

// ================================================================
// Private Functions
// ================================================================

static void ADXL345_SPI1_Init(void)
{
    /*
     * Disable analog function on used pins.
     *
     * RA3  -> CS
     * RB5  -> SDO1
     * RB8  -> SDI1
     * RB14 -> SCK1
     */
//    ANSELAbits.ANSA3 = 0;
//    ANSELBbits.ANSB5 = 0;
//    ANSELBbits.ANSB8 = 0;
    ANSELBbits.ANSB14 = 0;

    /*
     * CS is controlled manually as GPIO.
     * ADXL345 CS is active low.
     */
    ADXL345_CS_TRIS = 0;
    ADXL345_CS_High();

    /*
     * Pin directions.
     */
    TRISBbits.TRISB5 = 0;    // RB5  = SDO1 output
    TRISBbits.TRISB8 = 1;    // RB8  = SDI1 input
    TRISBbits.TRISB14 = 0;   // RB14 = SCK1 output

    /*
     * Unlock PPS.
     */
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    CFGCONbits.IOLOCK = 0;

    /*
     * PPS configuration.
     *
     * For many PIC32MX1xx/2xx devices:
     *
     * RB5  -> SDO1
     * RB8  -> SDI1
     *
     * IMPORTANT:
     * If your chip uses different PPS codes, check the datasheet.
     */
    RPB5Rbits.RPB5R = 0b0011;   // RB5 = SDO1
    SDI1Rbits.SDI1R = 0b0100;   // SDI1 = RB8

    /*
     * Lock PPS.
     */
    CFGCONbits.IOLOCK = 1;
    SYSKEY = 0x00000000;

    /*
     * Configure SPI1.
     */
    SPI1CONbits.ON = 0;

    /*
     * Clear receive buffer.
     */
    volatile uint8_t dummy;
    dummy = SPI1BUF;
    (void)dummy;

    /*
     * Clear overflow flag.
     */
    SPI1STATbits.SPIROV = 0;

    SPI1BRG = 99;
    SPI1CONbits.MSTEN = 1;      // Master mode
    SPI1CONbits.MODE16 = 0;     // 8-bit mode
    SPI1CONbits.MODE32 = 0;     // 8-bit mode

    SPI1CONbits.CKP = 1;
    SPI1CONbits.CKE = 0;

    /*
     * SMP:
     * 0 = input sampled at middle of data output time.
     * This is usually safe for SPI mode 3 at low speed.
     */
    SPI1CONbits.SMP = 0;

    /*
     * Turn on SPI1.
     */
    SPI1CONbits.ON = 1;
}

static uint8_t ADXL345_SPI1_Transfer(uint8_t data)
{
    SPI1BUF = data;

    while (!SPI1STATbits.SPIRBF) {
        ;
    }

    return (uint8_t)SPI1BUF;
}

static void ADXL345_CS_Low(void)
{
    ADXL345_CS_LAT = 0;
}

static void ADXL345_CS_High(void)
{
    ADXL345_CS_LAT = 1;
}


uint8_t ADXL345_TestReadID(void)
{
    uint8_t id;

    ADXL345_CS_High();

    for (volatile uint32_t i = 0; i < 1000; i++) {
        ;
    }

    ADXL345_CS_Low();

    for (volatile uint32_t i = 0; i < 1000; i++) {
        ;
    }

    ADXL345_SPI1_Transfer(0x80);
    id = ADXL345_SPI1_Transfer(0x00);

    for (volatile uint32_t i = 0; i < 1000; i++) {
        ;
    }

    ADXL345_CS_High();

    return id;
}