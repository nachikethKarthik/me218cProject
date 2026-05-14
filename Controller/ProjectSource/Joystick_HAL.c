#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "Joystick_HAL.h"
#include "PIC32_AD_Lib.h"

#define CHANNEL_SET  ((1u<<4)|(1u<<5))

static uint32_t ADCResults[2];
static uint32_t Y_Joystick;
static uint32_t X_Joystick;
bool Init_Joystick(void)
{
    ANSELBbits.ANSB2 = 1;
    ANSELBbits.ANSB3 = 1;

    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;

    return ADC_ConfigAutoScan(CHANNEL_SET);
}

uint32_t Read_X_Joystick(void)
{
    ADC_MultiRead(ADCResults);

    X_Joystick = ADCResults[1];
    return X_Joystick;
}

uint32_t Read_Y_Joystick(void)
{
    ADC_MultiRead(ADCResults);

    Y_Joystick = ADCResults[0];
    return Y_Joystick;
}