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
#include "Servo_HAL.h"
#include "PairingDisplay_HAL.h"
//#include "Potentiometer_HAL.h"  // Same in Joystick_HAL.h
/*----------------------------- Module Defines ----------------------------*/
#define XBEE_SEND_PERIOD_MS   200
#define DISPLAY_PERIOD_MS     200

/*---------------------------- Module Functions ---------------------------*/


/*---------------------------- Module Variables ---------------------------*/
static ControllerState_t CurrentState;


// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;
static uint32_t X_Joystick;
static uint32_t Y_Joystick;

static uint32_t boatselect;

static uint8_t boattarget;
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
  Servo_Init();
  Servo_SetAngle(0);
  
  SevenSeg_Init();
  
  //ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
  ES_Timer_InitTimer(DISPLAY_TIMER, DISPLAY_PERIOD_MS);
  
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool PostControllerService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

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
        
        //CurrentState = PairingState;
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
                uint8_t joy1 = Y_Joystick / 4;
                uint8_t joy2 = X_Joystick / 4;
                
                if (joy1 == 126){
                    joy1 == 127;
                }
                if (joy2 == 126){
                    joy2 == 127;
                }
                
                uint8_t digi = 0;
                XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM5_ADDR, joy1, joy2, digi);
                
                XBeeRxPacket_t rxPacket;
                if (XBeeHAL_Update())
                {
                    if (XBeeHAL_GetLastRxPacket(&rxPacket))
                    {
                        uint8_t charge = rxPacket.charge;
                        
                        if (charge == 0xFF)
                        {
                            DB_printf("Paired!!!!\n");
                        }else{
                            DB_printf("Charge = %d\n", charge);
                        }
                    }
                }
                
                DB_printf("Value of Joystick is x = %d, y = %d\n", X_Joystick, Y_Joystick);
            }
            else if ('d' == ThisEvent.EventParam){
                
                SevenSeg_DisplayDigit(3);

                
            }
            else if ('s' == ThisEvent.EventParam){
                
                Servo_SetAngle(150);
            }
            else if ('b' == ThisEvent.EventParam){
                uint32_t boatselect = Read_Potentiometer();
                SevenSeg_DisplayDigit(boatselect);
                DB_printf("boat select is %d\n",boatselect);
            }
        }
        break;
        case ES_TIMEOUT:
        {
            
            if (XBEE_TIMER == ThisEvent.EventParam){
                
                ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                
                X_Joystick = Read_X_Joystick();
                Y_Joystick = Read_Y_Joystick();
                uint8_t joy1 = Y_Joystick / 4;
                uint8_t joy2 = X_Joystick / 4;
                
                if (joy1 == 126){
                    joy1 == 127;
                }
                if (joy2 == 126){
                    joy2 == 127;
                }
                
                uint8_t digi = 0;
                XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM5_ADDR, joy1, joy2, digi);
                
                //XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM5_ADDR,  XBEE_MALLARD_TEAM5_ADDR);
                
                XBeeRxPacket_t rxPacket;
                if (XBeeHAL_Update())
                {
                    if (XBeeHAL_GetLastRxPacket(&rxPacket))
                    {
                        uint8_t charge = rxPacket.charge;
                        
                        if (charge == 0xFF)
                        {
                            DB_printf("Paired!!!!\n");
                        }else{
                            DB_printf("Charge = %d\n", charge);
                            Servo_SetAngle((uint8_t)charge);
                            
                            
                        }
                    }
                }
                
                //DB_printf("Value of Joystick is x = %d, y = %d\n", X_Joystick, Y_Joystick);
            }
        }
        break;
        case ES_PAIRINGBUTTON:
        {
           DB_printf("Pairing Button Pressed!\n"); 
        }
        break;
        
        case ES_TODRIVE:
        {
           DB_printf("Drive mode!\n"); 
        }
        break;
        
        case ES_TOREFUEL:
        {
           DB_printf("Refuel mode!\n"); 
        }
        break;
        
        
        default:
          ;
      }
    }
    break;
    case PairingState:
    {
        switch (ThisEvent.EventType)
        {
            case ES_PAIRINGBUTTON:
            {
                DB_printf("Pairing Button Pressed!\n"); 
                boatselect = Read_Potentiometer();
                SevenSeg_DisplayDigit(boatselect);
                
                switch (boatselect)
                {
                    case 1:
                        boattarget = 1;
                        ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                    break;
                    case 2:
                        boattarget = 2;
                        ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                    break;
                    case 3:
                        boattarget = 3;
                        ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                    break;
                    case 4:
                        boattarget = 4;
                        ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                    break;
                    case 5:
                        boattarget = 5;
                        ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                    break;
                    case 0:
                        //boattarget = 0;
                    break;
                    default:
                        ;
                }
            }
            break;
            
            case ES_TIMEOUT:
            {         
                //XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM5_ADDR,  XBEE_MALLARD_TEAM5_ADDR);
                if (XBEE_TIMER == ThisEvent.EventParam){
                    ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                    XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM5_ADDR,  XBEE_MALLARD_TEAM5_ADDR);
                    XBeeRxPacket_t rxPacket;
                    if (XBeeHAL_Update())
                    {
                        if (XBeeHAL_GetLastRxPacket(&rxPacket))
                        {
                            uint8_t charge = rxPacket.charge;
                            if (charge == 0xFF)
                            {
                                DB_printf("Paired!!!!\n");
                                // TODO: Turn on LED 
                                CurrentState = DrivingState;
                            }
                        }
                    }
                } else if (DISPLAY_TIMER == ThisEvent.EventParam){
                    boatselect = Read_Potentiometer();
                    SevenSeg_DisplayDigit(boatselect);
                    ES_Timer_InitTimer(DISPLAY_TIMER, DISPLAY_PERIOD_MS);
                }
            }
            break;
            default:
                ;
        }
    }   
    break;
    case DrivingState:
    {
        switch (ThisEvent.EventType)
        {
            case ES_TOREFUEL:
            {
                CurrentState = ChargingState;
            }
            break;
            
            case ES_TIMEOUT:
            { 
                if (XBEE_TIMER == ThisEvent.EventParam){
                    ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);

                    X_Joystick = Read_X_Joystick();
                    Y_Joystick = Read_Y_Joystick();
                    uint8_t joy1 = Y_Joystick / 4;
                    uint8_t joy2 = X_Joystick / 4;

                    if (joy1 == 126){
                        joy1 == 127;
                    }
                    if (joy2 == 126){
                        joy2 == 127;
                    }

                    uint8_t digi = 0; // TODO: Add button event
                    XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM5_ADDR, joy1, joy2, digi);

                    XBeeRxPacket_t rxPacket;
                    if (XBeeHAL_Update())
                    {
                        if (XBeeHAL_GetLastRxPacket(&rxPacket))
                        {
                            uint8_t charge = rxPacket.charge;

                            if (charge == 0xFF)
                            {
                                DB_printf("Paired!!!!\n");
                            }else{
                                DB_printf("Charge = %d\n", charge);
                                Servo_SetAngle((uint8_t)charge);
                            }
                        }
                    }

                    //DB_printf("Value of Joystick is x = %d, y = %d\n", X_Joystick, Y_Joystick);
                }
 
        }
        break;

        default:
          ;
        }
    }      
    break;
    
    case ChargingState:
    {
        switch (ThisEvent.EventType)
        {
            case ES_TODRIVE:
            {
                CurrentState = DrivingState;
            }
            break;
            case ES_TIMEOUT:
            {         
                if (XBEE_TIMER == ThisEvent.EventParam){
                    XBeeHAL_SendCharging(XBEE_QUACKRAFT_TEAM5_ADDR,  XBEE_MALLARD_TEAM5_ADDR); // TODO: add IMU event
                    ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                    XBeeRxPacket_t rxPacket;
                    if (XBeeHAL_Update())
                    {
                        if (XBeeHAL_GetLastRxPacket(&rxPacket))
                        {
                            uint8_t charge = rxPacket.charge;
                            if (charge == 0xFF)
                            {
                                DB_printf("Paired!!!!\n");
                                CurrentState = DrivingState;
                            }
                        }
                    }
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

ControllerState_t QueryTemplateFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

