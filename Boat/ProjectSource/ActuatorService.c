/****************************************************************************
 Module
   ActuatorService.c
 
 Description
   Service for the Quackraft that owns Timer2 (PWM time base) and the five
   Output Compare channels driving the thrusters and servos.
 
   Pin assignments (PIC32MX170F256B, peripheral pin select):
     OC1 -> RB3  (Left  thruster ESC)        RPB3R  = 0b0101
     OC2 -> RB5  (Right thruster ESC)        RPB5R  = 0b0101
     OC3 -> RB14 (Fauna gate servo)          RPB14R = 0b0101
     OC4 -> RB13 (Pairing-indicator servo)   RPB13R = 0b0101
     OC5 -> RA2  (Duck-smacker servo)        RPA2R  = 0b0110
 
   Timer2 is configured for 50 Hz (20 ms period). With PBCLK = 20 MHz and
   a 1:8 prescaler, the timer ticks at 2.5 MHz, so:
     PR2     = 49999    (20 ms period)
     1.0 ms  = 2500     (full reverse on bidirectional ESC)
     1.5 ms  = 3750     (ESC neutral / off)
     2.0 ms  = 5000     (full forward)
 
   Servo angle mapping (typical 0..180 deg hobby servo): (this will very likely need to be tuned for each servo)
     0   deg  ~ 1.0 ms pulse  ( 2500 OCxRS )
     90  deg  ~ 1.5 ms pulse  ( 3750 OCxRS )
     180 deg  ~ 2.0 ms pulse  ( 5000 OCxRS )
 
   Events consumed:
     ES_SET_THRUSTERS  param = (joy1 << 8) | joy2
     ES_SET_DIGI_OUT   param = digi byte (bit 0 = gate, bit 1 = smacker)
     ES_ALL_STOP       motors back to neutral, gate closed, smacker rest
     ES_SET_PAIR_IND   param = 0 unpaired (sideways), 1 paired (upright)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ActuatorService.h"
#include "dbprintf.h"


/*----------------------------- Module Defines ----------------------------*/
// Maximum thruster output, as a percentage of full PWM swing. Set to 60
// to cap the motors at +/-60% of their absolute max forward/reverse, which
// keeps them from over-thrusting at full joystick deflection.
#define THRUSTER_MAX_OUTPUT_PCT  50

// Timer2 period for 50 Hz PWM (20 ms) with PBCLK = 20 MHz, prescaler = 1:8
#define PWM_PERIOD_TICKS    49999u   // PR2 value -> 20 ms period
 


// Protocol byte interpretation
#define JOY_CENTER          0x7F
#define JOY_DEADBAND        5u      // ignore values within +/- this of center : TUNABLE PARAMETER

// Max magnitude of the signed thruster command after subtracting JOY_CENTER.
// A joystick byte of 0x00 yields -127 and 0xFF yields +128;
#define THRUSTER_MAX_CMD    127

// Digi byte bit positions (per class-wide protocol), these are bit masks
#define DIGI_BIT_GATE       0x01    // bit 0 = critter collect / gate
#define DIGI_BIT_SMACKER    0x02    // bit 1 = captain-duck smacker
 
// Servo position presets (in PWM ticks) [these will need tweaking for sure]]

// Pulse widths in timer ticks (timer = 2.5 MHz, so 1 tick = 0.4 us)
// 1.0 ms = 2500, 1.5 ms = 3750, 2.0 ms = 5000
#define PULSE_1MS_TICKS     2500u
#define PULSE_1P5MS_TICKS   3750u
#define PULSE_2MS_TICKS     5000u

// Gate: closed (down) vs open (up)
#define GATE_CLOSED_TICKS   PULSE_1MS_TICKS
#define GATE_OPEN_TICKS     PULSE_2MS_TICKS
 
// Smacker: rest vs deployed
#define SMACKER_REST_TICKS      PULSE_1MS_TICKS
#define SMACKER_DEPLOY_TICKS    PULSE_2MS_TICKS
 
// Pairing indicator: upright (paired) vs sideways (unpaired)
//   0 deg   = 1.0 ms pulse = upright   (paired)
//   90 deg  = 1.5 ms pulse = sideways  (unpaired)
#define PAIR_IND_PAIRED_TICKS    2050u//PULSE_1MS_TICKS
#define PAIR_IND_UNPAIRED_TICKS  PULSE_1P5MS_TICKS

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitTimer2For50HzPWM(void);
static void InitOutputCompareChannels(void);
static void SetThrusters(uint8_t joy1, uint8_t joy2);
static uint16_t SignedCmdToPulseTicks(int16_t cmd);
static void SetGate(bool gateOpen);
static void SetSmacker(bool smackerDeployed);
static void SetPairIndicator(bool isPaired);
static void AllStop(void);


/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitActuatorService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Sets up Timer2 and the 5 OC channels, drives all outputs to safe/idle
     starting positions, and posts ES_INIT to self.
 Notes

 Author
    karthi24
****************************************************************************/
bool InitActuatorService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  
  // Configure Timer2 (PWM time base) before we touch the OC modules.
  InitTimer2For50HzPWM();
 
  // Configure all 5 OC channels and set safe starting outputs.
  InitOutputCompareChannels();
 
  // Drive everything to a known, safe idle state right at boot.
  AllStop();
  SetPairIndicator(false);  // boot in the "unpaired" position
 
  DB_printf("\rActuatorService init complete\r\n");
  
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
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
     PostActuatorService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
    karthi24
****************************************************************************/
bool PostActuatorService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunActuatorService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
    karthi24
****************************************************************************/
ES_Event_t RunActuatorService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
    {
      // Nothing more to do; init already configured the hardware.
      break;
    }
 
    case ES_SET_THRUSTERS:
    {
      // EventParam packs joy1 in the high byte, joy2 in the low byte.
      uint8_t joy1 = (uint8_t)((ThisEvent.EventParam >> 8) & 0xFF);
      uint8_t joy2 = (uint8_t)(ThisEvent.EventParam & 0xFF);
      DB_printf("\rreceived  ES_SET_THRUSTERS with joy1 = %d and joy2 = %d\r\n",joy1,joy2);
      SetThrusters(joy1, joy2);
      break;
    }
 
    case ES_SET_DIGI_OUT:
    {
      uint8_t digi = (uint8_t)(ThisEvent.EventParam & 0xFF);
      // (digi & DIGI_BIT_GATE) produces a uint8_t which is then converted to bool using the != 0 
      SetGate((digi & DIGI_BIT_GATE) != 0);
      SetSmacker((digi & DIGI_BIT_SMACKER) != 0);
      DB_printf("\rreceived  ES_SET_DIGI_OUT, controlling the gate and \r\n");
      
      break;
    }
 
    case ES_ALL_STOP:
    {
      AllStop();
      break;
    }
 
    case ES_SET_PAIR_IND:
    {
      SetPairIndicator(ThisEvent.EventParam != 0);
      DB_printf("\rreceived  ES_SET_PAIR_IND, controlling the pairing indicator servo setting it to %d\r\n",ThisEvent.EventParam);
      break;
    }
 
    default:
      break;
  }
  
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
   InitTimer2For50HzPWM
 
 Description
   Configure Timer2 to roll over every 20 ms (50 Hz) so it serves as the
   PWM time base for all OC channels feeding the thrusters/servos.
 
   PBCLK = 20 MHz. With TCKPS = 0b011 (1:8 prescale), Timer2 ticks at
   2.5 MHz, so PR2 = (0.020 * 2_500_000) - 1 = 49999.
****************************************************************************/
static void InitTimer2For50HzPWM(void)
{
    // Make sure Timer2 is off while we set it up.
    //clear the on bit to disable the timer
    T2CONbits.ON = 0;

    // Internal peripheral clock source (TCS = 0), gate off (TGATE = 0)
    T2CONbits.TCS   = 0;
    T2CONbits.TGATE = 0; // disable gated time accumulation
    // 16-bit timer mode (T32 = 0)
    T2CONbits.T32 = 0; // dont combine timers to form 32 bit timers

    // Prescaler 1:8  ->  Timer2 clock = 20 MHz / 8 = 2.5 MHz
    T2CONbits.TCKPS = 0b011;
    
    // Reset the running count. Load and clear the timer register
    TMR2 = 0;
    
    // Period register: rollover at 20 ms. Load the period register with the 16 bit match value
    PR2 = PWM_PERIOD_TICKS;
    
    //clear the interrupt flag
    IFS0CLR = _IFS0_T2IF_MASK;

    // Turn Timer2 on.
    T2CONbits.ON = 1;
}

/****************************************************************************
 Function
   InitOutputCompareChannels
 
 Description
   Configure all 5 OC channels for PWM mode without fault pin, sourced from
   Timer2. Also sets the TRIS bits to output and configures the PPS mux for
   each pin. Outputs start at safe neutral pulse widths.
****************************************************************************/
static void InitOutputCompareChannels(void)
{
    // ---- Step 1: turn off all OC modules before we touch them ----
    OC1CON = 0x0000;
    OC2CON = 0x0000;
    OC3CON = 0x0000;
    OC4CON = 0x0000;
    OC5CON = 0x0000;

    // ---- Step 2: configure pins as digital outputs ----
    // OC1 -> RB3
    TRISBbits.TRISB3  = 0;
    ANSELBbits.ANSB3  = 0;   // RB3 is analog-capable; disable analog
    // OC2 -> RB5
    TRISBbits.TRISB5  = 0;
    // OC3 -> RB14
    TRISBbits.TRISB14 = 0;
    ANSELBbits.ANSB14 = 0;   // RB14 is analog-capable; disable analog
    // OC4 -> RB13
    TRISBbits.TRISB13 = 0;
    ANSELBbits.ANSB13 = 0;   // RB13 is analog-capable; disable analog
    // OC5 -> RA2
    TRISAbits.TRISA2  = 0;

    // ---- Step 3: PPS output: assign each pin to its OC peripheral ----
    RPB3R  = 0b0101;   // OC1 on RB3
    RPB5R  = 0b0101;   // OC2 on RB5
    RPB14R = 0b0101;   // OC3 on RB14
    RPB13R = 0b0101;   // OC4 on RB13
    RPA2R  = 0b0110;   // OC5 on RA2

    // ---- Step 4: load initial neutral pulse widths ----
    // Thrusters: neutral (ESC off)
    OC1RS = PULSE_1P5MS_TICKS;
    OC2RS = PULSE_1P5MS_TICKS;
    // Gate: closed
    OC3RS = GATE_CLOSED_TICKS;
    // Pair indicator: unpaired
    OC4RS = PAIR_IND_UNPAIRED_TICKS;
    // Smacker: rest
    OC5RS = SMACKER_REST_TICKS;
    // Thrusters: neutral (ESC off)
    OC1R  = PULSE_1P5MS_TICKS;
    OC2R  = PULSE_1P5MS_TICKS;
    // Gate: closed
    OC3R  = GATE_CLOSED_TICKS;
    // Pair indicator: unpaired
    OC4R  = PAIR_IND_UNPAIRED_TICKS;
    // Smacker: rest
    OC5R  = SMACKER_REST_TICKS;
    
    // Interrupts not required
    
    // ---- Step 5: configure mode and timer source, then enable each OC ----
    // OCxCON = 0x0006 == OCM<2:0>=110 (PWM, no fault), OCTSEL=0 (Timer2)
    OC1CONbits.OCM = 0b110;
    OC2CONbits.OCM  = 0b110;
    OC3CONbits.OCM  = 0b110;
    OC4CONbits.OCM  = 0b110;
    OC5CONbits.OCM  = 0b110;

    // Set the ON bit (OCxCON<15>) for each
    OC1CONbits.ON = 1;
    OC2CONbits.ON = 1;
    OC3CONbits.ON = 1;
    OC4CONbits.ON = 1;
    OC5CONbits.ON = 1;
}

/****************************************************************************
 Function
   SetThrusters
 
 Description
   Combines the protocol's forward/back axis (Joy1)
   with the left/right axis (Joy2) into per-thruster commands. This lets
   the operator simultaneously translate and rotate (e.g. "forward and
   slightly left" by pushing Joy1 up and Joy2 left at the same time).
 
   Math:
     forward = (int16_t)joy1 - JOY_CENTER       // range ~ -127..+127
     turn    = (int16_t)joy2 - JOY_CENTER       // range ~ -127..+127
     left    = forward + turn
     right   = forward - turn
 
   With this mixing:
     - Pure forward (joy1 high, joy2 centered):  left = right = +N -> both
       motors forward, boat translates forward.
     - Pure right turn (joy1 centered, joy2 right): left = +N, right = -N
       -> boat rotates in place clockwise (turns right).
     - Forward + left (joy1 high, joy2 low): left smaller, right larger
       -> boat curves forward-left.
 
****************************************************************************/
static void SetThrusters(uint8_t joy1, uint8_t joy2)
{
  int16_t forward = (int16_t)joy1 - (int16_t)JOY_CENTER;
  int16_t turn    = (int16_t)joy2 - (int16_t)JOY_CENTER;
 
  int16_t leftCmd  = forward - turn;
  int16_t rightCmd = forward + turn;
 
  OC1RS = SignedCmdToPulseTicks(-leftCmd);    // left thruster
  OC2RS = SignedCmdToPulseTicks(rightCmd);   // right thruster
}


/****************************************************************************
 Function
   SignedCmdToPulseTicks
 
 Description
   Map a signed thruster command (range roughly -127..+127) to a PWM pulse
   width in Timer2 ticks (PULSE_1MS_TICKS..PULSE_2MS_TICKS, center
   PULSE_1P5MS_TICKS).
 
     cmd = -127 -> 1.0 ms (full reverse)
     cmd =    0 -> 1.5 ms (off / neutral)
     cmd = +127 -> 2.0 ms (full forward)
 
   Applies a small deadband around zero so noise / imperfectly-centered
   sticks don't creep the motors.
****************************************************************************/
static uint16_t SignedCmdToPulseTicks(int16_t cmd)
{
  // Clamp into the valid signed range.
  if (cmd >  THRUSTER_MAX_CMD)  cmd =  THRUSTER_MAX_CMD;
  if (cmd < -THRUSTER_MAX_CMD)  cmd = -THRUSTER_MAX_CMD;
 
  // Deadband around zero -> neutral pulse.
  if (cmd > -((int16_t)JOY_DEADBAND) && cmd < (int16_t)JOY_DEADBAND)
  {
    return PULSE_1P5MS_TICKS;
  }
 
  // Linear map (-THRUSTER_MAX_CMD,+THRUSTER_MAX_CMD) to (PULSE_1MS_TICKS,PULSE_2MS_TICKS), centered at 1.5 ms.
  //   ticks = PULSE_1P5MS_TICKS + (cmd * half_span) / THRUSTER_MAX_CMD
  
  // Full half-span is 1250 ticks (1.5 ms +/- 0.5 ms). Scale down by the
// THRUSTER_MAX_OUTPUT_PCT to cap how aggressive the thrusters can get.
    int32_t halfSpan = ((int32_t)(PULSE_2MS_TICKS - PULSE_1MS_TICKS) / 2) *
                   THRUSTER_MAX_OUTPUT_PCT / 100;

  int32_t offset   = ((int32_t)cmd * halfSpan) / (int32_t)THRUSTER_MAX_CMD;
  int32_t ticks    = (int32_t)PULSE_1P5MS_TICKS + offset;
 
  return (uint16_t)ticks;
}

/****************************************************************************
 Function
   SetGate
 
 Description
   Drive the fauna gate servo to either the open or closed preset.
****************************************************************************/
static void SetGate(bool gateOpen)
{
  OC3RS = gateOpen ? GATE_OPEN_TICKS : GATE_CLOSED_TICKS;
}

/****************************************************************************
 Function
   SetSmacker
 
 Description
   Drive the duck-smacker servo to either rest or deployed.
****************************************************************************/
static void SetSmacker(bool smackerDeployed)
{
  OC5RS = smackerDeployed ? SMACKER_DEPLOY_TICKS : SMACKER_REST_TICKS;
}

/****************************************************************************
 Function
   SetPairIndicator
 
 Description
   Rotate the team-number indicator. Upright = paired, sideways = unpaired.
****************************************************************************/
static void SetPairIndicator(bool isPaired)
{
  OC4RS = isPaired ? PAIR_IND_PAIRED_TICKS : PAIR_IND_UNPAIRED_TICKS;
}

/****************************************************************************
 Function
   AllStop
 
 Description
   Drive thrusters to neutral and digi-controlled actuators to rest.
   Does NOT touch the pairing indicator (that is managed separately by
   ES_SET_PAIR_IND so it survives ES_ALL_STOP events).
****************************************************************************/
static void AllStop(void)
{
  OC1RS = PULSE_1P5MS_TICKS;   // left thruster off
  OC2RS = PULSE_1P5MS_TICKS;   // right thruster off
  OC3RS = GATE_CLOSED_TICKS;   // gate closed
  OC5RS = SMACKER_REST_TICKS;  // smacker at rest
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

