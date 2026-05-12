#include "Controller_Communication_HAL.h"
#include "UART_HAL.h"

#define XBEE_TX_FRAME_SIZE      13
#define XBEE_RX_FRAME_SIZE      10

#define XBEE_START_DELIMITER    0x7E
#define XBEE_API_ID_TX_16       0x01
#define XBEE_FRAME_ID_DISABLED  0x00
#define XBEE_OPTIONS_NO_ACK     0x01

// TX length = 0x0009
#define XBEE_TX_LENGTH_MSB      0x00
#define XBEE_TX_LENGTH_LSB      0x09

// RX length = 0x0006
#define XBEE_RX_LENGTH_MSB      0x00
#define XBEE_RX_LENGTH_LSB      0x06

static XBeeRxPacket_t lastRxPacket;

// Parser state
typedef enum {
    XBEE_PARSE_WAIT_START = 0,
    XBEE_PARSE_LENGTH_MSB,
    XBEE_PARSE_LENGTH_LSB,
    XBEE_PARSE_PAYLOAD
} XBeeParseState_t;

static XBeeParseState_t parseState = XBEE_PARSE_WAIT_START;
static uint8_t rxFrame[XBEE_RX_FRAME_SIZE];
static uint8_t rxIndex = 0;
static uint16_t expectedLength = 0;

static uint8_t XBeeHAL_CalculateChecksum(const uint8_t *frame, uint8_t frameSize);
static bool XBeeHAL_ParseCompleteRxFrame(const uint8_t *frame);
static void XBeeHAL_SendPacket(const XBeeTxPacket_t *packet);


void XBeeHAL_Init(void)
{
    UARTHAL_Init();

    lastRxPacket.destinationAddress = 0;
    lastRxPacket.charge = 0;
    lastRxPacket.valid = false;

    parseState = XBEE_PARSE_WAIT_START;
    rxIndex = 0;
    expectedLength = 0;
}

// Send driving signal
void XBeeHAL_SendDriving(uint16_t quackraftAddress, uint8_t joy1, uint8_t joy2, uint8_t digi)
{
    XBeeTxPacket_t packet;

    packet.destinationAddress = quackraftAddress;
    packet.status = XBEE_STATUS_DRIVING;
    packet.joy1 = joy1;
    packet.joy2 = joy2;
    packet.digi = digi;

    XBeeHAL_SendPacket(&packet);
}


void XBeeHAL_SendIdle(uint16_t quackraftAddress)
{
    XBeeHAL_SendDriving(quackraftAddress, XBEE_JOY_CENTER, XBEE_JOY_CENTER, 0x00);
}


void XBeeHAL_SendCharging(uint16_t quackraftAddress)
{
    XBeeTxPacket_t packet;

    packet.destinationAddress = quackraftAddress;
    packet.status = XBEE_STATUS_CHARGING;
    packet.joy1 = 0x00;
    packet.joy2 = 0x00;
    packet.digi = 0x00;

    XBeeHAL_SendPacket(&packet);
}


void XBeeHAL_SendPairing(uint16_t quackraftAddress,
                         uint16_t mallardAddress)
{
    XBeeTxPacket_t packet;

    packet.destinationAddress = quackraftAddress;
    packet.status = XBEE_STATUS_PAIRING;

    packet.joy1 = (uint8_t)((mallardAddress >> 8) & 0xFF);
    packet.joy2 = (uint8_t)(mallardAddress & 0xFF);

    packet.digi = 0x00;

    XBeeHAL_SendPacket(&packet);
}

// Send the packet 
static void XBeeHAL_SendPacket(const XBeeTxPacket_t *packet)
{
    if (packet == 0) {
        return;
    }

    uint8_t frame[XBEE_TX_FRAME_SIZE];

    frame[0]  = XBEE_START_DELIMITER;
    frame[1]  = XBEE_TX_LENGTH_MSB;
    frame[2]  = XBEE_TX_LENGTH_LSB;
    frame[3]  = XBEE_API_ID_TX_16;
    frame[4]  = XBEE_FRAME_ID_DISABLED;
    frame[5]  = (uint8_t)((packet->destinationAddress >> 8) & 0xFF);
    frame[6]  = (uint8_t)(packet->destinationAddress & 0xFF);
    frame[7]  = XBEE_OPTIONS_NO_ACK;
    frame[8]  = packet->status;
    frame[9]  = packet->joy1;
    frame[10] = packet->joy2;
    frame[11] = packet->digi;
    frame[12] = XBeeHAL_CalculateChecksum(frame, XBEE_TX_FRAME_SIZE);

    UARTHAL_SendBytes(frame, XBEE_TX_FRAME_SIZE);
}

static uint8_t XBeeHAL_CalculateChecksum(const uint8_t *frame, uint8_t frameSize)
{
    uint16_t sum = 0;

    for (uint8_t i = 3; i < frameSize - 1; i++) {
        sum += frame[i];
    }

    return (uint8_t)(0xFF - (sum & 0xFF));
}

bool XBeeHAL_Update(void)
{
    uint8_t byteRead;
    bool gotPacket = false;

    while (UARTHAL_ReadByte(&byteRead)) {

        switch (parseState) {

            case XBEE_PARSE_WAIT_START:
                if (byteRead == XBEE_START_DELIMITER) {
                    rxFrame[0] = byteRead;
                    rxIndex = 1;
                    parseState = XBEE_PARSE_LENGTH_MSB;
                }
                break;

            case XBEE_PARSE_LENGTH_MSB:
                rxFrame[rxIndex++] = byteRead;
                expectedLength = ((uint16_t)byteRead) << 8;
                parseState = XBEE_PARSE_LENGTH_LSB;
                break;

            case XBEE_PARSE_LENGTH_LSB:
                rxFrame[rxIndex++] = byteRead;
                expectedLength |= byteRead;

                if (expectedLength == 0x0006) {
                    parseState = XBEE_PARSE_PAYLOAD;
                } else {
                    parseState = XBEE_PARSE_WAIT_START;
                    rxIndex = 0;
                    expectedLength = 0;
                }
                break;

            case XBEE_PARSE_PAYLOAD:
                rxFrame[rxIndex++] = byteRead;

                if (rxIndex >= XBEE_RX_FRAME_SIZE) {
                    if (XBeeHAL_ParseCompleteRxFrame(rxFrame)) {
                        gotPacket = true;
                    }

                    parseState = XBEE_PARSE_WAIT_START;
                    rxIndex = 0;
                    expectedLength = 0;
                }
                break;

            default:
                parseState = XBEE_PARSE_WAIT_START;
                rxIndex = 0;
                expectedLength = 0;
                break;
        }
    }

    return gotPacket;
}


static bool XBeeHAL_ParseCompleteRxFrame(const uint8_t *frame)
{
    if (frame == 0) {
        return false;
    }

    if (frame[0] != XBEE_START_DELIMITER) {
        return false;
    }

    if (frame[1] != XBEE_RX_LENGTH_MSB || frame[2] != XBEE_RX_LENGTH_LSB) {
        return false;
    }

    if (frame[3] != XBEE_API_ID_TX_16) {
        return false;
    }

    if (frame[4] != XBEE_FRAME_ID_DISABLED) {
        return false;
    }

    if (frame[7] != XBEE_OPTIONS_NO_ACK) {
        return false;
    }

    // verify checksum.
    uint16_t sum = 0;
    for (uint8_t i = 3; i <= 8; i++) {
        sum += frame[i];
    }

    sum += frame[9];

    if ((sum & 0xFF) != 0xFF) {
        return false;
    }

    lastRxPacket.destinationAddress = ((uint16_t)frame[5] << 8) | frame[6];
    lastRxPacket.charge = frame[8];
    lastRxPacket.valid = true;

    return true;
}


bool XBeeHAL_GetLastRxPacket(XBeeRxPacket_t *packet)
{
    if (packet == 0) {
        return false;
    }

    if (!lastRxPacket.valid) {
        return false;
    }

    *packet = lastRxPacket;
    return true;
}


bool XBeeHAL_IsPairingSuccess(void)
{
    if (!lastRxPacket.valid) {
        return false;
    }

    return lastRxPacket.charge == XBEE_PAIRING_SUCCESS_CHARGE;
}