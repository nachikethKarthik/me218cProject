/*----------------------------- Include Files -----------------------------*/
#include <xc.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
#include "ControllerService.h"
#include "Controller_Communication_HAL.h"
#include "UART_HAL.h"
#include "Joystick_HAL.h"
/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/


/*---------------------------- Module Variables ---------------------------*/
static ControllerState_t CurrentState;


// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;
static uint32_t X_Joystick;
static uint32_t Y_Joystick;
/*------------------------------ Module Code ------------------------------*/
bool InitControllerService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  DB_printf("Controller Service Start!\n");
  MyPriority = Priority;
  CurrentState = TestState;
  ThisEvent.EventType = ES_INIT;
  
  XBeeHAL_Init();
  Init_Joystick();
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostTemplateFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostControllerService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunControllerService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  switch (CurrentState)
  {
    case InitPState:
    {
      if (ThisEvent.EventType == ES_INIT)
      {

        CurrentState = TestState;
      }
    }
    break;

    case TestState:
    {
      //DB_printf("TestState!\n");
      switch (ThisEvent.EventType)
      {
        case ES_NEW_KEY:
        {
            //DB_printf("1\n");
            if('c' == ThisEvent.EventParam)
            {
                DB_printf("c pressed\n");
                XBeeHAL_SendCharging(XBEE_QUACKRAFT_TEAM5_ADDR);
            }
            else if('i' == ThisEvent.EventParam)
            {
                DB_printf("i pressed\n");
                XBeeHAL_SendIdle(XBEE_QUACKRAFT_TEAM5_ADDR);
            }
            else if ('p' == ThisEvent.EventParam)
            {
                DB_printf("p pressed\n");
                XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM5_ADDR,  XBEE_MALLARD_TEAM5_ADDR);
            }
            else if ('j' == ThisEvent.EventParam)
            {
                //DB_printf("j pressed\n");
                X_Joystick = Read_X_Joystick();
                Y_Joystick = Read_Y_Joystick();
                DB_printf("Value of Joystick is x = %d, y = %d\n", X_Joystick, Y_Joystick);
            }
        }
        break;

        default:
          ;
      }
    }
    break;
    default:
      ;
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
ControllerState_t QueryTemplateFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

