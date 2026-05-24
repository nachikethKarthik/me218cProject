/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef ControllerService_H
#define ControllerService_H

// Event Definitions
#include <stdint.h>
#include <stdbool.h>

#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"

#include "ES_Events.h"
#include "ES_Port.h" 
// typedefs for the states
// State definitions for use with the query function
extern uint8_t ButtonValue;
extern bool GateControl;

typedef enum
{
  InitPState, TestState, PairingState, ChargingState, DrivingState, _1UnlockPress,
  _2UnlockPresses, Locked
}ControllerState_t;

// Public Function Prototypes

bool InitControllerService(uint8_t Priority);
bool PostControllerService(ES_Event_t ThisEvent);
ES_Event_t RunControllerService(ES_Event_t ThisEvent);
ControllerState_t QueryTemplateSM(void);

#endif /* FSMTemplate_H */

