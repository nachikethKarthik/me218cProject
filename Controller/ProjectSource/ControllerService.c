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
#include "ADXL345_HAL.h"
#include "RGB_HAL.h"
//#include "Potentiometer_HAL.h"  // Same in Joystick_HAL.h
/*----------------------------- Module Defines ----------------------------*/
#define XBEE_SEND_PERIOD_MS     200
#define DISPLAY_PERIOD_MS       200
#define DOUBLECLICK_PERIOD_MS   500

#define UPPER_X      40
#define LOWER_X      -40
#define UPPER_Y      40
#define LOWER_Y      -40
#define UPPER_Z      300
#define LOWER_Z      200
/*---------------------------- Module Functions ---------------------------*/


/*---------------------------- Module Variables ---------------------------*/
static ControllerState_t CurrentState;


// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;
static uint32_t X_Joystick;
static uint32_t Y_Joystick;

static uint32_t boatselect;

static uint8_t boattarget;

static uint8_t gatevalue;
static bool gatestate = false;
static bool stickstate = false;


static ADXL345_RawData_t raw;

static bool doubleclick_control = false; 
static bool doubletimer_control = false;

bool GateControl = false;
/*------------------------------ Module Code ------------------------------*/
bool InitControllerService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  DB_printf("Controller Service Start!\n");
  MyPriority = Priority;
  CurrentState = InitPState;
  ThisEvent.EventType = ES_INIT;
  
  XBeeHAL_Init();
  Init_Joystick();
  Servo_Init();
  RGB_Init();
  
  Servo_SetAngle(0);
  
  SevenSeg_Init();
  
  bool ok = ADXL345_Init();
  if (ok){
      DB_printf("IMU OK\n");
  }
  
//  uint8_t id;
//
//  ADXL345_Init();
//
//  id = ADXL345_TestReadID();
//
//  DB_printf("ADXL ID = %d\r\n", id);
  
  
  
  //ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
  
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
/*----------------------------- InitState ----------------------------*/
      case InitPState:
    {
      if (ThisEvent.EventType == ES_INIT)
      {

        //CurrentState = TestState;
        
        CurrentState = PairingState;
        ES_Timer_InitTimer(DISPLAY_TIMER, DISPLAY_PERIOD_MS);
      }
    }
    break;

/*----------------------------- TestState ----------------------------*/
    case TestState:
    {
      DB_printf("TestState!\n");
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
                if (gatestate == true){
                    digi += 0x01;
                }else{
                    // None
                }
                
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
                
                Servo_SetAngle(200);
            }
            else if ('b' == ThisEvent.EventParam){
                uint32_t boatselect = Read_Potentiometer();
                SevenSeg_DisplayDigit(boatselect);
                DB_printf("boat select is %d\n",boatselect);
            }
            else if ('r' == ThisEvent.EventParam){
                ADXL345_ReadRaw(&raw);
                DB_printf("Raw data is x = %d, y = %d, z = %d\n", raw.x, raw.y, raw.z);
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
                            Servo_SetAngle(200 - (uint8_t)charge);
                            
                            
                        }
                    }
                }
                
            } else if (GATECONTROL_TIMER == ThisEvent.EventParam){
                GateControl = false;
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
        
        case ES_GATEBUTTON_PRESS:
        {
            DB_printf("Gate Button Pressed!\n"); 
            if (gatestate == true){
                gatestate = false;
            }else{
                gatestate = true;
            }
        }
        break;
        
        case ES_GATEBUTTON_RELEASE:
        {
            DB_printf("Gate Button Released!\n"); 
        }
        break;
        default:
          ;
      }
    }
    break;
    
    
/*----------------------------- PairingState ----------------------------*/
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
                        boattarget = 0;
                    break;
                    default:
                        ;
                }
            }
            break;
            
            case ES_TIMEOUT:
            {         
                if (XBEE_TIMER == ThisEvent.EventParam){
                    ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);

                    switch (boattarget)
                    {
                        case 1:
                            XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM1_ADDR, XBEE_MALLARD_TEAM5_ADDR);
                        break;
                        case 2:
                            XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM2_ADDR, XBEE_MALLARD_TEAM5_ADDR);
                        break;
                        case 3:
                            XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM3_ADDR, XBEE_MALLARD_TEAM5_ADDR);
                        break;
                        case 4:
                            XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM4_ADDR, XBEE_MALLARD_TEAM5_ADDR);
                        break;
                        case 5:
                            XBeeHAL_SendPairing(XBEE_QUACKRAFT_TEAM5_ADDR, XBEE_MALLARD_TEAM5_ADDR);
                        break;
                        case 0:
                            //boattarget = 0;
                        break;
                        default:
                            ;
                    }

                    XBeeRxPacket_t rxPacket;
                    if (XBeeHAL_Update())
                    {
                        if (XBeeHAL_GetLastRxPacket(&rxPacket))
                        {
                            uint8_t charge = rxPacket.charge;
                            if (charge == 0xFF)
                            {
                                DB_printf("Paired!!!!\n");
                                RGB_ON();
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
    
/*----------------------------- DrivingState ----------------------------*/
    case DrivingState:
    {
        //DB_printf("DrivingState!\n");
        switch (ThisEvent.EventType)
        {
            case ES_TOREFUEL:
            {
                CurrentState = ChargingState;
            }
            break;
            
            case ES_GATEBUTTON_PRESS:
            {
                if (doubleclick_control == false){
                    doubleclick_control = true;
                    ES_Timer_InitTimer(DOUBLECLICK_TIMER, DOUBLECLICK_PERIOD_MS);
                }else{
                    ES_Event_t ThisEvent;
                    ThisEvent.EventType = ES_DOUBLECLICK;
                    ES_PostAll(ThisEvent);
                    doubleclick_control = false;
                    //ES_Timer_StopTimer(DOUBLECLICK_TIMER);
                }
                
//                if (gatestate == true){
//                    gatestate = false;
//                }else{
//                    gatestate = true;
//                }
            }
            break;
            case ES_DOUBLECLICK:
            {
                if (stickstate == true){
                    stickstate = false;
                    DB_printf("Stick off!\n");
                }else{
                    stickstate = true;
                    DB_printf("Stick on!\n");
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
                    if (gatestate == true){
                        digi += 1;
                    }else{
                        // None
                    }
                    if (stickstate == true){
                        digi += 2;
                    }else{
                        // None
                    }
                    
                    //XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM5_ADDR, joy1, joy2, digi);

                    switch (boattarget)
                    {
                        case 1:
                            XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM1_ADDR, joy1, joy2, digi);
                        break;
                        case 2:
                            XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM2_ADDR, joy1, joy2, digi);
                        break;
                        case 3:
                            XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM3_ADDR, joy1, joy2, digi);
                        break;
                        case 4:
                            XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM4_ADDR, joy1, joy2, digi);
                        break;
                        case 5:
                            XBeeHAL_SendDriving(XBEE_QUACKRAFT_TEAM5_ADDR, joy1, joy2, digi);
                        break;
                        case 0:
                            //boattarget = 0;
                        break;
                        default:
                            ;
                    }
                    
                    
                    
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
                                //DB_printf("Charge = %d\n", charge);
                                Servo_SetAngle(200 - (uint8_t)charge);
                            }
                        }
                    }

                    //DB_printf("Value of   is x = %d, y = %d\n", X_Joystick, Y_Joystick);
                }else if (GATECONTROL_TIMER == ThisEvent.EventParam){
                    GateControl = false;
                }else if (DOUBLECLICK_TIMER == ThisEvent.EventParam){
                    if(doubleclick_control == true){
                        doubleclick_control = false;
                        if (gatestate == true){
                            gatestate = false;
                            DB_printf("Gate off!\n");
                        }else{
                            gatestate = true;
                            DB_printf("Gate on!\n");
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

/*----------------------------- ChargingState ----------------------------*/    
    case ChargingState:
    {
        doubleclick_control = false;
        //DB_printf("ChargingState!\n");
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
                    
                    ADXL345_ReadRaw(&raw);
                        if (raw.x >= UPPER_X || raw.x <= LOWER_X || raw.y >= UPPER_Y || raw.y <= LOWER_Y || raw.z >= UPPER_Z || raw.z <= LOWER_Z)
                        {
                            switch (boattarget)
                            {
                                case 1:
                                    XBeeHAL_SendCharging(XBEE_QUACKRAFT_TEAM1_ADDR);
                                break;
                                case 2:
                                    XBeeHAL_SendCharging(XBEE_QUACKRAFT_TEAM2_ADDR);
                                break;
                                case 3:
                                    XBeeHAL_SendCharging(XBEE_QUACKRAFT_TEAM3_ADDR);
                                break;
                                case 4:
                                    XBeeHAL_SendCharging(XBEE_QUACKRAFT_TEAM4_ADDR);
                                break;
                                case 5:
                                    XBeeHAL_SendCharging(XBEE_QUACKRAFT_TEAM5_ADDR);
                                break;
                                case 0:
                                    //boattarget = 0;
                                break;
                                default:
                                    ;
                            }
                        }else{
                            switch (boattarget)
                            {
                                case 1:
                                    XBeeHAL_SendIdle(XBEE_QUACKRAFT_TEAM1_ADDR);
                                break;
                                case 2:
                                    XBeeHAL_SendIdle(XBEE_QUACKRAFT_TEAM2_ADDR);
                                break;
                                case 3:
                                    XBeeHAL_SendIdle(XBEE_QUACKRAFT_TEAM3_ADDR);
                                break;
                                case 4:
                                    XBeeHAL_SendIdle(XBEE_QUACKRAFT_TEAM4_ADDR);
                                break;
                                case 5:
                                    XBeeHAL_SendIdle(XBEE_QUACKRAFT_TEAM5_ADDR);
                                break;
                                case 0:
                                    //boattarget = 0;
                                break;
                                default:
                                    ;
                            }
                        }
                    ES_Timer_InitTimer(XBEE_TIMER, XBEE_SEND_PERIOD_MS);
                    XBeeRxPacket_t rxPacket;
                    if (XBeeHAL_Update())
                    {
                        if (XBeeHAL_GetLastRxPacket(&rxPacket))
                        {
                            uint8_t charge = rxPacket.charge;
                            if (charge == 0xFF)
                            {
                                // None
                            }else{
                                DB_printf("Charge = %d\n", charge);
                                Servo_SetAngle(200 - (uint8_t)charge);
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

