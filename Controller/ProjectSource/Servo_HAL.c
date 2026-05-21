#include "Servo_HAL.h"
#include <xc.h>
#include <sys/attribs.h>
#include <stdint.h>

#define TICS_PER_MS      2500U
#define SERVO_MIN_TICKS  1500U
#define SERVO_MAX_TICKS  4800U

void Servo_Init(void)
{
    TRISBbits.TRISB15 = 0;
    
    T3CONbits.ON = 0;
    T3CONbits.TCKPS = 0b011;
    TMR3 = 0;
    PR3 = 49999;
    T3CONbits.ON = 1;
    
    // OC1 - RB15
    OC1CONbits.ON = 0;
    OC1R = 0;
    OC1RS = 0;
    OC1CONbits.OCTSEL = 1;
    OC1CONbits.OCM = 0b110;
    RPB15Rbits.RPB15R = 0b0101;
    OC1CONbits.ON = 1;
}

// Servo_PressureLevel - OC1
void Servo_SetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    uint16_t pw = (uint16_t)(SERVO_MIN_TICKS +((uint16_t)angle * (uint16_t)(SERVO_MAX_TICKS - SERVO_MIN_TICKS)) / 180UL);
    if (pw < SERVO_MIN_TICKS) pw = SERVO_MIN_TICKS;
    if (pw > SERVO_MAX_TICKS) pw = SERVO_MAX_TICKS;
    OC1RS = pw;
}


void Servo_SetPulseWidth(uint16_t pw){
    
    if (pw < SERVO_MIN_TICKS) pw = SERVO_MIN_TICKS;
    if (pw > SERVO_MAX_TICKS) pw = SERVO_MAX_TICKS;
    OC1RS = pw;
}

