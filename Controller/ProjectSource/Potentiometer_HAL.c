#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "Potentiometer_HAL.h"
#include "PIC32_AD_Lib.h"

#define CHANNEL_SET  (1u<<12)

static uint32_t ADCResults[1];
static uint32_t BoatSelect;

bool Init_Potentiometer(void)
{
    ANSELBbits.ANSB12 = 1;
    TRISBbits.TRISB12 = 1;

    return ADC_ConfigAutoScan(CHANNEL_SET);
}

uint32_t Read_Potentiometer(void)
{
    ADC_MultiRead(ADCResults);

    BoatSelect = ADCResults[1] / 200;
    return BoatSelect;
}
