/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/
/****************************************************************************
 Module
   XBeeService.h
 
 Description
   Public interface for the XBeeService. This service owns UART2 and the
   class-wide communications protocol between the Quackraft and a Mallard
   Module. It implements the pairing FSM (Unpaired <-> Paired) and the
   steam-pressure accounting required by the project spec.
 
   The UART2 RX interrupt service routine lives in this module. It assembles incoming bytes into XBee API frames
   (validating start delimiter, length, and checksum) and posts an
   ES_RX_FRAME event to this service when a valid frame is available.
 
   Events :
     ES_INIT     (framework: do initial transition into Unpaired)
     ES_RX_FRAME (from UART2 ISR: decoded frame is in the fresh buffer)
     ES_TIMEOUT  (PAIRING_WATCHDOG_TIMER: lost paired Mallard Module)
 
   Events generated (posted to ActuatorService):
     ES_SET_THRUSTERS, ES_SET_DIGI_OUT, ES_ALL_STOP, ES_SET_PAIR_IND
****************************************************************************/
#ifndef XBEE_SERVICE_H
#define XBEE_SERVICE_H

#include "ES_Types.h"

// Public Function Prototypes

bool        InitXBeeService(uint8_t Priority);
bool        PostXBeeService(ES_Event_t ThisEvent);
ES_Event_t  RunXBeeService(ES_Event_t ThisEvent);

#endif /* XBEE_SERVICE_H */

