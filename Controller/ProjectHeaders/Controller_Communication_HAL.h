#ifndef XBEE_HAL_H
#define XBEE_HAL_H

#include <stdint.h>
#include <stdbool.h>

#define XBEE_QUACKRAFT_TEAM1_ADDR   0x2081
#define XBEE_QUACKRAFT_TEAM2_ADDR   0x2082
#define XBEE_QUACKRAFT_TEAM3_ADDR   0x2087
#define XBEE_QUACKRAFT_TEAM4_ADDR   0x2084
#define XBEE_QUACKRAFT_TEAM5_ADDR   0x2086

#define XBEE_MALLARD_TEAM1_ADDR     0x2181
#define XBEE_MALLARD_TEAM2_ADDR     0x2182
#define XBEE_MALLARD_TEAM3_ADDR     0x2187
#define XBEE_MALLARD_TEAM4_ADDR     0x2184
#define XBEE_MALLARD_TEAM5_ADDR     0x2186

#define XBEE_STATUS_DRIVING         0x00
#define XBEE_STATUS_CHARGING        0x01
#define XBEE_STATUS_PAIRING         0x02

#define XBEE_JOY_MIN                0x00
#define XBEE_JOY_CENTER             0x7F
#define XBEE_JOY_MAX                0xFF

#define XBEE_PAIRING_SUCCESS_CHARGE 0xFF

typedef struct {
    uint16_t destinationAddress;
    uint8_t status;
    uint8_t joy1;
    uint8_t joy2;
    uint8_t digi;
} XBeeTxPacket_t;

typedef struct {
    uint16_t destinationAddress;
    uint8_t charge;
    bool valid;
} XBeeRxPacket_t;

void XBeeHAL_Init(void);

void XBeeHAL_SendDriving(uint16_t quackraftAddress,
                         uint8_t joy1,
                         uint8_t joy2,
                         uint8_t digi);

void XBeeHAL_SendIdle(uint16_t quackraftAddress);

void XBeeHAL_SendCharging(uint16_t quackraftAddress);

void XBeeHAL_SendPairing(uint16_t quackraftAddress,
                         uint16_t mallardAddress);
                         
bool XBeeHAL_Update(void);

bool XBeeHAL_GetLastRxPacket(XBeeRxPacket_t *packet);

bool XBeeHAL_IsPairingSuccess(void);

#endif