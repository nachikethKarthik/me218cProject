#include "PairingDisplay_HAL.h"
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>


#define SR_CLK_LAT     LATAbits.LATA0
#define SR_CLK_TRIS    TRISAbits.TRISA0
#define SR_CLK_ANSEL   ANSELAbits.ANSA0

#define SR_LATCH_LAT   LATAbits.LATA1
#define SR_LATCH_TRIS  TRISAbits.TRISA1
#define SR_LATCH_ANSEL ANSELAbits.ANSA1

#define SR_SER_LAT     LATAbits.LATA2
#define SR_SER_TRIS    TRISAbits.TRISA2

#define SEG_A   (1u << 0)
#define SEG_B   (1u << 1)
#define SEG_C   (1u << 2)
#define SEG_D   (1u << 3)
#define SEG_E   (1u << 4)
#define SEG_F   (1u << 5)
#define SEG_G   (1u << 6)
#define SEG_DP  (1u << 7)

static uint8_t CurrentPattern = 0x00;

static const uint8_t DigitTable[10] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,          // 0
    SEG_B | SEG_C,                                          // 1
    SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,                  // 2
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,                  // 3
    SEG_B | SEG_C | SEG_F | SEG_G,                          // 4
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,                  // 5
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,          // 6
    SEG_A | SEG_B | SEG_C,                                  // 7
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,  // 8
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G           // 9
};

static void SevenSeg_ShortDelay(void)
{
    volatile int i;
    for (i = 0; i < 10; i++) {
        ;
    }
}


static void SevenSeg_PulseClock(void)
{
    SR_CLK_LAT = 0;
    SevenSeg_ShortDelay();

    SR_CLK_LAT = 1;
    SevenSeg_ShortDelay();

    SR_CLK_LAT = 0;
    SevenSeg_ShortDelay();
}


static void SevenSeg_PulseLatch(void)
{
    SR_LATCH_LAT = 0;
    SevenSeg_ShortDelay();

    SR_LATCH_LAT = 1;
    SevenSeg_ShortDelay();

    SR_LATCH_LAT = 0;
    SevenSeg_ShortDelay();
}

static void SevenSeg_ShiftOut(uint8_t data)
{
    int8_t i;

    for (i = 7; i >= 0; i--) {
        if (data & (1u << i)) {
            SR_SER_LAT = 1;
        } else {
            SR_SER_LAT = 0;
        }

        SevenSeg_PulseClock();
    }

    SevenSeg_PulseLatch();
}


bool SevenSeg_Init(void)
{
    SR_CLK_ANSEL =   0;
    SR_LATCH_ANSEL = 0;

    SR_CLK_TRIS = 0;
    SR_LATCH_TRIS = 0;
    SR_SER_TRIS = 0;

    SR_CLK_LAT = 0;
    SR_LATCH_LAT = 0;
    SR_SER_LAT = 0;

    CurrentPattern = 0x00;
    SevenSeg_ShiftOut(CurrentPattern);

    return true;
}


void SevenSeg_DisplayRaw(uint8_t pattern)
{
    CurrentPattern = pattern;
    SevenSeg_ShiftOut(CurrentPattern);
}


void SevenSeg_DisplayDigit(uint8_t digit)
{
    if (digit > 9) {
        SevenSeg_DisplayOff();
        return;
    }

    CurrentPattern = DigitTable[digit];
    SevenSeg_ShiftOut(CurrentPattern);
}


void SevenSeg_DisplayOff(void)
{
    CurrentPattern = 0x00;
    SevenSeg_ShiftOut(CurrentPattern);
}


void SevenSeg_DisplayDP(bool on)
{
    if (on) {
        CurrentPattern |= SEG_DP;
    } else {
        CurrentPattern &= ~SEG_DP;
    }

    SevenSeg_ShiftOut(CurrentPattern);
}