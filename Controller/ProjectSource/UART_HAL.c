#include "UART_HAL.h"
#include <xc.h>
#include <sys/attribs.h>

#define PBCLK_HZ        20000000UL
#define UART_BAUD       9600UL

// UxBRG = PBCLK / (16 * Baud) - 1
#define UART_BRG_VALUE  ((PBCLK_HZ / (16 * UART_BAUD)) - 1)

#define UART_RX_BUFFER_SIZE 64

static volatile uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint8_t rxHead = 0;
static volatile uint8_t rxTail = 0;

static void UARTHAL_ConfigPins(void);


void UARTHAL_Init(void)
{
    UARTHAL_ConfigPins();

    rxHead = 0;
    rxTail = 0;

    // Turn UART off
    U2MODEbits.ON = 0;
    U2STAbits.UTXEN = 0;
    U2STAbits.URXEN = 0;

    // Baud rate
    U2MODEbits.BRGH = 0;
    U2BRG = UART_BRG_VALUE;

    // 8N1 format
    U2MODEbits.PDSEL = 0b00;      // 8-bit data, no parity
    U2MODEbits.STSEL = 0;         // 1 stop bit

    // Normal UART mode
    U2MODEbits.SIDL = 0;
    U2MODEbits.IREN = 0;
    U2MODEbits.RTSMD = 0;
    U2MODEbits.UEN = 0b00;
    U2MODEbits.WAKE = 0;
    U2MODEbits.LPBACK = 0;
    U2MODEbits.ABAUD = 0;
    U2MODEbits.RXINV = 0;

    // RX interrupt when each character is received
    U2STAbits.URXISEL = 0b00;

    // TX interrupt not used
    IEC1bits.U2TXIE = 0;

    // Clear error and interrupt flags
    if (U2STAbits.OERR) {
        U2STAbits.OERR = 0;
    }

    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    IFS1bits.U2EIF = 0;

    // Configure RX interrupt priority
    IPC9bits.U2IP = 2;
    IPC9bits.U2IS = 0;

    // Enable RX interrupt
    IEC1bits.U2RXIE = 1;

    // Turn UART on
    U2MODEbits.ON = 1;

    // Enable transmitter and receiver
    U2STAbits.UTXEN = 1;
    U2STAbits.URXEN = 1;
}


static void UARTHAL_ConfigPins(void)
{

    // Disable analog function on pins
    //ANSELBbits.ANSB10 = 0;
    //ANSELBbits.ANSB11 = 0;

    // Set directions
    TRISBbits.TRISB10 = 0;     // TX output
    TRISBbits.TRISB11 = 1;    // RX input

    // Map U2RX to RPB11
    U2RXRbits.U2RXR = 0b0011;

    // Map U2TX to RPB10
    RPB10Rbits.RPB10R = 0b0010;
}

void UARTHAL_SendByte(uint8_t data)
{
    while (U2STAbits.UTXBF) {
        ;
    }

    U2TXREG = data;
}

void UARTHAL_SendBytes(const uint8_t *data, uint16_t length)
{
    if (data == 0) {
        return;
    }

    for (uint16_t i = 0; i < length; i++) {
        UARTHAL_SendByte(data[i]);
    }
}

// void UARTHAL_SendString(const char *str)
// {
//     if (str == 0) {
//         return;
//     }

//     while (*str != '\0') {
//         UARTHAL_SendByte((uint8_t)(*str));
//         str++;
//     }
// }


bool UARTHAL_IsRxAvailable(void)
{
    return rxHead != rxTail;
}

bool UARTHAL_ReadByte(uint8_t *data)
{
    if (data == 0) {
        return false;
    }

    if (rxHead == rxTail) {
        return false;
    }

    *data = rxBuffer[rxTail];
    rxTail = (rxTail + 1) % UART_RX_BUFFER_SIZE;

    return true;
}

void UARTHAL_ClearRxBuffer(void)
{
    rxHead = 0;
    rxTail = 0;
}

// =======================================================
// UART1 RX Interrupt
// =======================================================

void __ISR(_UART_2_VECTOR, IPL2SOFT) UART2_ISR(void)
{
    if (IFS1bits.U2RXIF) {

        while (U2STAbits.URXDA) {

            uint8_t data = U2RXREG;

            uint8_t nextHead = (rxHead + 1) % UART_RX_BUFFER_SIZE;

            if (nextHead != rxTail) {
                rxBuffer[rxHead] = data;
                rxHead = nextHead;
            }
        }

        IFS1bits.U2RXIF = 0;
    }

    // Clear overrun error
    if (U2STAbits.OERR) {
        U2STAbits.OERR = 0;
    }
}