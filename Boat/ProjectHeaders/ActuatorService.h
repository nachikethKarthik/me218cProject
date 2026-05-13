/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ACTUATOR_SERVICE_H
#define ACTUATOR_SERVICE_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitTemplateService(uint8_t Priority);
bool        PostActuatorService(ES_Event_t ThisEvent);
ES_Event_t RunTemplateService(ES_Event_t ThisEvent);

#endif /* ACTUATOR_SERVICE_H */

