// RGB_HAL.c
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "RGB_HAL.h"

void RGB_Init(void)
{
    ANSELBbits.ANSB13 = 0;
    TRISBbits.TRISB13 = 0;
    LATBbits.LATB13 = 0;
}

void RGB_ON(void)
{
    LATBbits.LATB13 = 1;
}

void RGB_OFF(void)
{
    LATBbits.LATB13 = 0;
}
