/****************************************************************************
 Module
   XBeeService.c
 
 Description
   Owns UART2 + the class-wide ME218C communications protocol on the
   Quackraft side. Maintains the Quackcraft pairing state machine and tracks steam
   pressure. The UART2 RX interrupt service routine is implemented in
   this file and assembles incoming bytes into XBee API frames before
   posting ES_RX_FRAME to this service.
 
   UART2 configuration:
     - U2RX on RB11 (U2RXR = 0b0011)
     - U2TX on RB10 (RPB10R = 0b0010)
     - 9600 baud, 8N1, BRGH = 0 (standard speed)
       BRG = PBCLK / (16 * baud) - 1 = 20MHz / (16 * 9600) - 1 = 129
       (gives ~9615 baud => 0.16% error)
     - RX interrupt enabled at priority 7
     - TX is sent by polled blocking write (no TX interrupt)
     - URXISEL = 0b00 -> RX interrupt asserts whenever any byte is in FIFO

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "XBeeService.h"
#include <xc.h> // for interrupts
#include <sys/attribs.h> // for interrupts
#include <string.h>
#include "dbprintf.h" // for print debugs
#include "ActuatorService.h"

/*----------------------------- Module Defines ----------------------------*/
/* ----- UART2 -------------------------------------------------------- */
// Baud rate divisor for 9600 baud with PBCLK = 20 MHz, BRGH = 0:
//   BRG = (PBCLK / (16 * baud)) - 1 = 20_000_000 / 153_600 - 1 = 129
#define UART_BRG_9600           129u
/* ----- XBee API frame constants ------------------------------------- */
#define XBEE_START_DELIM        0x7E
#define XBEE_API_TX16           0x01    // TX request, 16-bit address, API identifier
#define XBEE_API_RX16           0x81    // RX packet, 16-bit address, API identifier
 
// Max size (in bytes) of frame data we'll store while assembling an incoming frame. from API ID to Digi.
// The largest RX frame we expect is an RX packet wrapping our
// 4-byte protocol payload (status/joy1/joy2/digi). That's a frame-data length
// of ~9 bytes, so 32 bytes is plenty of margin.
#define RX_FRAME_BUFFER_SIZE    32u
/* ----- ME218C class-wide protocol constants ------------------------- */
// Status byte values (Mallard Module -> Quackraft)
#define STATUS_DRIVING          0x00
#define STATUS_CHARGING         0x01
#define STATUS_PAIRING          0x02
 
// Digi byte bits (low 2 bits standardized; bits 2-7 team-specific)
#define DIGI_BIT_GATE           0x01    // bit 0 = critter collect / gate
#define DIGI_BIT_SMACKER        0x02    // bit 1 = captain-duck smacker
 
// Joystick neutral and deadband used for steam-cost detection
#define JOY_CENTER              0x7F
#define JOY_DEADBAND            5
 
// Steam (a.k.a. fuel) bookkeeping
#define STEAM_MAX               150     // per comms-committee protocol
#define STEAM_CHARGE_INCREMENT  6       // each charging msg adds this
 
// Pair-ack sentinel value sent in the charge byte during pairing handshake
#define CHARGE_BYTE_PAIR_ACK    0xFF
 
// Pairing watchdog: the Quackraft must unpair if it hears nothing from its
// paired Mallard Module for this many milliseconds.
#define PAIRING_WATCHDOG_MS     4000u

/* ----- Quackraft / Mallard Module address tables -------------------- */
// (These are the class-wide addresses. The Quackraft's own address is
//  baked into the build via TEAM_NUMBER below.)
#define TEAM_NUMBER             5   
#define MY_QUACKRAFT_ADDR_HIGH  0x20
#define MY_QUACKRAFT_ADDR_LOW   (0x80u + TEAM_NUMBER)
// Mallard Module addresses are 0x2181..0x2185
/* ----- Incoming frame byte offsets (XBee 0x81 RX packet) or index values ------------ */
// After the start delimiter + length bytes are consumed by the isr,
// the bytes stored in our rxFrame[] buffer are the frame-data bytes:
//   [0] API ID  (should be 0x81 for RX 16-bit)
//   [1] Source addr MSB will be 0x21
//   [2] Source addr LSB will be 0x8(1-5))
//   [3] RSSI
//   [4] Options
//   [5-8]  RF data
//   [9]  checksum 
#define RXF_API_ID              0
#define RXF_SRC_ADDR_HI         1
#define RXF_SRC_ADDR_LO         2
#define RXF_RSSI                3
#define RXF_OPTIONS             4
#define RXF_PAYLOAD_START       5
 
// Within the 4-byte payload (Mallard Module -> Quackraft):
#define PAYLOAD_STATUS          0
#define PAYLOAD_JOY1            1
#define PAYLOAD_JOY2            2
#define PAYLOAD_DIGI            3

/*---------------------------- Type Definitions ---------------------------*/
typedef enum
{
  QC_UNPAIRED,
  QC_PAIRED
} XBeeState_t;
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitUART2(void);
static void HandleRxFrame(const uint8_t *frame, uint8_t frameLen);
static void SendResponse(uint8_t chargeByte);
static void TransmitFrame(const uint8_t *frame, uint8_t len);
static void HandleDrivingMessage(uint8_t joy1, uint8_t joy2, uint8_t digi);
static void HandleChargingMessage(void);
static void HandlePairingMessage(uint8_t pairingJoy1, uint8_t pairingJoy2);
static uint8_t ComputeSteamCost(uint8_t joy1, uint8_t joy2, uint8_t digi);
static void GoUnpaired(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static XBeeState_t  CurrentState = QC_UNPAIRED;
// Currently-paired Mallard Module address. Valid only while CurrentState is
// QC_PAIRED. Stored separately as two bytes for direct use in TX frame.
static uint8_t      PairedMM_AddrHi = 0x00;
static uint8_t      PairedMM_AddrLo = 0x00;
// Current steam pressure (0..STEAM_MAX). Re-Charged on pairing after 4s timer expiration and on each
// successful charging message; drained per actionable Driving message.
static uint8_t      Steam = 0;

// ---- RX buffers used between the UART2 ISR and the Run function ----
// The ISR assembles incoming bytes into rxFrameWorking. When a complete
// checksum-valid frame is built, the ISR copies it into rxFrameReady,
// stores its length in rxFrameReadyLen, and posts ES_RX_FRAME. The Run
// function reads exclusively from rxFrameReady.
//
// Splitting "working" from "ready" means the Run function can read its
// buffer without worrying about ongoing assembly of the next frame
// scribbling on top of it.

static volatile uint8_t  rxFrameReady[RX_FRAME_BUFFER_SIZE];
static volatile uint8_t  rxFrameReadyLen = 0;

 
// ---- Per-byte frame assembly state, lives entirely inside the ISR ----
static volatile bool     InFrame      = false;
static volatile uint8_t  FrameIdx     = 0;     // index of next byte to store
static volatile uint16_t WorkingFrameLen  = 0;     // parsed from length bytes within the isr
static volatile uint8_t  LengthHi     = 0;
static volatile uint8_t  RunningSum   = 0;
static volatile uint8_t  rxFrameWorking[RX_FRAME_BUFFER_SIZE]; // working frame assembled within ISR, copied into FrameReady buffer upon receiving full frame

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitXBeeService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Sets up UART2 (including the RX interrupt), and posts ES_INIT to self.
 Notes

 Author
    karthi24
****************************************************************************/
bool InitXBeeService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // Reset all module state.
  CurrentState     = QC_UNPAIRED;
  PairedMM_AddrHi  = 0x00;
  PairedMM_AddrLo  = 0x00;
  Steam            = 0;
  InFrame          = false;
  FrameIdx         = 0;
  WorkingFrameLen      = 0;
  RunningSum       = 0;
  rxFrameReadyLen  = 0;
 
  // Configure UART2 (and its RX interrupt). This globally enables interrupts.
  InitUART2();
 
  DB_printf("\rXBeeService Init complete (Quackraft team %d)\r\n", TEAM_NUMBER);
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
     PostXBeeService

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
bool PostXBeeService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
 * RunXBeeService

Parameters
 * ES_Event_t : the event to process

Returns
 * ES_Event_t : ES_NO_EVENT on success, ES_ERROR otherwise

Description
 * Top-level dispatcher for the XBeeService FSM. RX frames and framework timer timeouts both flow through here.

Notes
 * To be added later

Author
 * karthi24
****************************************************************************/
ES_Event_t RunXBeeService(ES_Event_t ThisEvent)
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
      // Nothing extra to do on init; we boot in QR_UNPAIRED.
      break;
    }
 
    case ES_RX_FRAME:
    {
      // The ISR just posted a complete, checksum-valid frame for us. The
      // frame data is sitting in rxFrameReady.
        DB_printf("\rNew frame received !\r\n");
        uint8_t        frameLen = rxFrameReadyLen;
        const uint8_t *frame    = (const uint8_t *)rxFrameReady;
        HandleRxFrame(frame, frameLen);
        break;
    }
 
    case ES_TIMEOUT:
    {
      if (ThisEvent.EventParam == PAIRING_WATCHDOG_TIMER)
      {
        // 4 s of silence from the paired Mallard Module -> unpair.
        if (CurrentState == QC_PAIRED)
        {
          DB_printf("\rPairing watchdog expired, going Unpaired\r\n");
          GoUnpaired();
        }
      }
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
 ISR
     UART2_ISR
 
 Description
     Drains every byte currently in the RX FIFO and feeds it through a
     per-byte frame-assembly state machine. When a complete valid frame
     arrives, copies it from the working buffer into the ready buffer and
     posts ES_RX_FRAME so the Run function will pick it up.
 
     IMPORTANT: per the PIC32 family reference manual, on UART modules
     with an 8-deep FIFO the RX interrupt flag stays asserted as long as
     the condition selected by URXISEL is true. With URXISEL = 00, this
     means we must drain the FIFO completely before clearing the IFS flag,
     otherwise the flag will simply re-assert and we'll re-enter forever.
****************************************************************************/
void __ISR(_UART_2_VECTOR, IPL7SOFT) UART2_ISR(void)
{
  // Drain the entire RX FIFO before clearing the flag.
  while (U2STAbits.URXDA)
  {
    uint8_t b = (uint8_t)U2RXREG;
//    DB_printf("\rReading the receive buffer !\r\n");
    if (!InFrame)
    {
      // Looking for the XBee start delimiter.
      if (b == XBEE_START_DELIM)
      {
//        DB_printf("\rInside a frame!\r\n");
        InFrame    = true;
        FrameIdx   = 0;
        RunningSum = 0;
        
        // Don't store the start delimiter itself; it's not part of
        // the length or the checksum.
      }
      // Any other byte while not in-frame is junk and silently dropped.
    }
    else
    {
      // We're mid-frame. The very first two bytes after the delimiter are
      // the length (MSB then LSB) and are NOT included in the checksum.
      // After that, every byte (frame data + final checksum byte) IS
      // included in the running sum.
      if (FrameIdx == 0)
      {
        LengthHi = b;
        FrameIdx++;
      }
      else if (FrameIdx == 1)
      {
        WorkingFrameLen = ((uint16_t)LengthHi << 8) | (uint16_t)b;
        FrameIdx++;
        // Bounds-check: if the announced length is impossibly large,
        // abandon this frame. We need room for ExpectedLen data bytes
        // PLUS one checksum byte; both go in rxFrameWorking, but only
        // the data bytes go into the "frame data" we hand to the FSM.
        if (WorkingFrameLen == 0 || WorkingFrameLen > (RX_FRAME_BUFFER_SIZE - 1))
        {
          InFrame  = false;
          FrameIdx = 0;
        }
      }
      else
      {
        // FrameIdx >= 2 here. The next ExpectedLen bytes are the frame
        // data; the one after that is the checksum byte.
        uint16_t dataIdx = (uint16_t)FrameIdx - 2u; // 0-based into frame data
        if (dataIdx < WorkingFrameLen)
        {
          rxFrameWorking[dataIdx] = b;
          RunningSum += b;
          FrameIdx++;
        }
        else
        {
          // This byte is the checksum byte.
          RunningSum += b;
          FrameIdx++;
 
          if ((uint8_t)(RunningSum & 0xFFu) == 0xFFu)
          {
            // Valid frame! Copy from the working buffer into the ready
            // buffer so the Run function can read it without contention
            // with the next frame's ongoing assembly.
            for (uint16_t i = 0; i < WorkingFrameLen; i++)
            {
              rxFrameReady[i] = rxFrameWorking[i];
            }
            rxFrameReadyLen = (uint8_t)WorkingFrameLen;
 
            // Post ES_RX_FRAME from ISR context.
            ES_Event_t evt;
            evt.EventType  = ES_RX_FRAME;
            evt.EventParam = 0;
            PostXBeeService(evt);
          }
          // Whether checksum was valid or not, frame is complete.
          InFrame  = false;
          FrameIdx = 0;
        }
      }
    }
  }
 
  // Now that the FIFO is empty, it's safe to clear the IFS flag.
  IFS1CLR = _IFS1_U2RXIF_MASK;
}

/****************************************************************************
 Function
     InitUART2
 
 Description
     Following the standard 11-step PIC32 UART init sequence. Enables both
     transmitter and receiver, and turns on only the RX interrupt at
     priority 7. TX is performed by polled blocking writes (no TX interrupt).
****************************************************************************/
static void InitUART2(void)
{
  // STEP 1: Make sure UART2 is disabled by clearing the ON bit
  U2MODEbits.ON = 0;
 
  // STEP 2: Map U2RX to a physical pin (RB11)
  U2RXR = 0b0011;            // U2RX <- RPB11
 
  // Configure RB11 as a digital input
  TRISBbits.TRISB11 = 1;     // input direction
  // (RB11 has no ANSEL bit on this part, so nothing to clear there.)
 
  // STEP 3: Map U2TX to a physical pin (RB10)
  RPB10R = 0b0010;           // RPB10 -> U2TX
 
  // Configure RB10 as a digital output
  TRISBbits.TRISB10 = 0;     // output
  // (RB10 also has no ANSEL bit on this part.)
 
  // STEP 4: Configure UxMODE to clear SIDL, IREN, RTSMD, UEN, WAKE,
  //         LPBACK, ABAUD, RXINV bits
  U2MODEbits.SIDL   = 0;     // continue in idle mode
  U2MODEbits.IREN   = 0;     // IrDA disabled
  U2MODEbits.RTSMD  = 0;     // flow control mode (irrelevant, UEN = 00)
  U2MODEbits.UEN    = 0;     // only TX and RX pins used (no flow control)
  U2MODEbits.WAKE   = 0;     // wake-up on start bit disabled
  U2MODEbits.LPBACK = 0;     // loopback disabled
  U2MODEbits.ABAUD  = 0;     // auto-baud disabled
  U2MODEbits.RXINV  = 0;     // RX idle state is high (non-inverted)
 
  // STEP 5: Configure UxMODE to choose baud clock speed
  U2MODEbits.BRGH   = 0;     // standard speed mode (16x baud clock)
 
  // STEP 6: Configure UxMODE for number of data bits, stop bits and parity
  U2MODEbits.PDSEL  = 0;     // 8-bit data, no parity
  U2MODEbits.STSEL  = 0;     // 1 stop bit
 
  // STEP 7: Configure UxSTA to clear UTXINV, UTXBRK and ADDEN bits
  U2STAbits.UTXINV  = 0;     // TX idle state is high (normal)
  U2STAbits.UTXBRK  = 0;     // break transmission disabled
  U2STAbits.ADDEN   = 0;     // address detect mode disabled
 
  // STEP 8: Configure UTXISEL and URXISEL to choose interrupt trigger
  //         conditions. We don't use the TX interrupt, but set it to a
  //         sensible value anyway. URXISEL = 00 -> RX flag is asserted
  //         whenever there is at least one byte in the RX FIFO.
  U2STAbits.UTXISEL = 0b00;
  U2STAbits.URXISEL = 0b00;
 
  // STEP 9: Set UTXEN and URXEN to enable transmitter and receiver
  U2STAbits.UTXEN   = 1;     // enable transmitter
  U2STAbits.URXEN   = 1;     // enable receiver
 
  // STEP 10: Write the baud rate constant to UxBRG
  U2BRG = UART_BRG_9600;
 
  // STEP 11: Enable UART2 by setting the ON bit in UxMODE
  U2MODEbits.ON     = 1;
 
  // ---- Configure RX interrupt (priority 7) ----
  // Disable TX interrupt explicitly (we are using polled blocking TX).
  IEC1CLR        = _IEC1_U2TXIE_MASK;
  // Clear any stale RX flag, then set priority/subpriority and enable.
  IFS1CLR        = _IFS1_U2RXIF_MASK;
  IPC9bits.U2IP  = 7;        // interrupt priority 7 (highest)
  IPC9bits.U2IS  = 0;        // subpriority 0
  IEC1SET        = _IEC1_U2RXIE_MASK;

  __builtin_enable_interrupts();
}

/****************************************************************************
 Function
     HandleRxFrame
 
 Description
     Top-level RX dispatch. Validates that the frame is an 0x81 RX packet
     , looks at the source address relative to our pairing
     state, and dispatches to the appropriate status-byte handler.
 Notes
 *  
 * This is the function that is called every time an ES_RxFrame event is posted and this event is posted every time we get a valid frame.
 * This function is responsible for keeping the 4 second watchdog timer alive
****************************************************************************/
static void HandleRxFrame(const uint8_t *frame, uint8_t frameLen)
{
  
 
  // Only handle RX 16-bit packets; ignore other API frame types.
  // Check if API identifier is 0x81
  if (frame[RXF_API_ID] != XBEE_API_RX16)
  {
    DB_printf("\rNew frame has the wrong API identifier !\r\n");
    return;
  }
  
//   [0] API ID  (should be 0x81 for RX 16-bit)
//   [1] Source addr MSB will be 0x21
//   [2] Source addr LSB will be 0x8(1-5))
//   [3] RSSI
//   [4] Options
//   [5-8]  RF data
//   [9]  checksum 
 
  uint8_t srcHi  = frame[RXF_SRC_ADDR_HI];
  uint8_t srcLo  = frame[RXF_SRC_ADDR_LO];
  uint8_t status = frame[RXF_PAYLOAD_START + PAYLOAD_STATUS];
  uint8_t joy1   = frame[RXF_PAYLOAD_START + PAYLOAD_JOY1];
  uint8_t joy2   = frame[RXF_PAYLOAD_START + PAYLOAD_JOY2];
  uint8_t digi   = frame[RXF_PAYLOAD_START + PAYLOAD_DIGI];
 
  if (CurrentState == QC_UNPAIRED)
  {
    // Only Pairing messages are interesting in this state.
    if (status == STATUS_PAIRING)
    {
      // Save the source address (the XBee 16-bit addr the MM transmitted from)
      // as our paired Mallard Module.
      PairedMM_AddrHi = srcHi;
      PairedMM_AddrLo = srcLo;
 
      // New pairing -> full tank of steam.
      Steam = STEAM_MAX;
 
      DB_printf("\rPaired with MM %u %u\r\n",
                PairedMM_AddrHi, PairedMM_AddrLo);
 
      // Tell the actuator service to indicate "paired".
      ES_Event_t evt;
      evt.EventType  = ES_SET_PAIR_IND;
      evt.EventParam = 1;
      PostActuatorService(evt);
 
      // Respond with pair-ack sentinel.
      SendResponse(CHARGE_BYTE_PAIR_ACK);
 
      // Start the 4 s watchdog and transition to Paired.
      ES_Timer_InitTimer(PAIRING_WATCHDOG_TIMER, PAIRING_WATCHDOG_MS);
      CurrentState = QC_PAIRED;
 
      // (Joy1/Joy2 in a Pairing message carry the MM's own address per
      //  the protocol. We don't currently need them since the XBee frame
      //  already gives us the source address)
    }// end if status is paired
    // Anything else in Unpaired -> ignore.
  }// end if current state is QC_UNPAIRED  and new frame received
  else // CurrentState == QC_PAIRED
  {
    // Loyalty: ignore any message that isn't from our paired MM.
    if (srcHi != PairedMM_AddrHi || srcLo != PairedMM_AddrLo)
    {
      return;
    }
 
    // Whatever the message is, our paired MM is still talking to us, so
    // restart the unpair watchdog.
    ES_Timer_InitTimer(PAIRING_WATCHDOG_TIMER, PAIRING_WATCHDOG_MS);
 
    switch (status)
    {
      case STATUS_DRIVING:
        DB_printf("\rDriving message received, handling it now\r\n");
        HandleDrivingMessage(joy1, joy2, digi);
        break;
 
      case STATUS_CHARGING:
        DB_printf("\rCharging message received, handling it now\r\n");
        HandleChargingMessage();
        break;
 
      case STATUS_PAIRING:
        // Re-pairing ping from the same MM. Per protocol: do NOT refill
        // steam; respond with 0xFF in the charge byte.
        DB_printf("\rPairing message received, handling it now\r\n");
        HandlePairingMessage(joy1, joy2);
        break;
 
      default:
        // Unknown status byte; ignore.
        break;
    }//end switch on status byte
  }// end if current state is QC_PAIRED and new frame received
}

/****************************************************************************
 Function
     SendResponse
 
 Description
     Build and transmit a 10-byte XBee API TX Request (0x01) frame whose
     RF data carries our single Charge byte, sent to the currently-paired
     Mallard Module.
 
     Layout:
        [0] 0x7E           start delimiter
        [1] 0x00           length MSB
        [2] 0x06           length LSB  (frame data = 6 bytes)
        [3] 0x01           API ID (TX 16-bit)
        [4] 0x00           frame ID (0 = no TX Status response)
        [5] dest addr MSB  (paired MM addr)
        [6] dest addr LSB
        [7] 0x01           Options: disable ACK
        [8] chargeByte     our payload
        [9] checksum
****************************************************************************/
static void SendResponse(uint8_t chargeByte)
{
  uint8_t frame[10];
  frame[0] = XBEE_START_DELIM;
  frame[1] = 0x00;
  frame[2] = 0x06;
  frame[3] = XBEE_API_TX16; // 0x01
  frame[4] = 0x00;                  // frame ID = 0, no TX status wanted
  frame[5] = PairedMM_AddrHi;
  frame[6] = PairedMM_AddrLo;
  frame[7] = 0x01;                  // disable ACK
  frame[8] = chargeByte; // send 0xFF as the charge byte for pairing message responses alone
 
  // Checksum: sum of bytes [3..8] (API ID through last data byte), low 8
  // bits subtracted from 0xFF.
  uint8_t sum = 0;
  for (uint8_t i = 3; i <= 8; i++)
  {
    sum += frame[i];
  }
  frame[9] = (uint8_t)(0xFF - sum);
 
  TransmitFrame(frame, sizeof(frame));
}

/****************************************************************************
 Function
     TransmitFrame
 
 Description
     Polled blocking write of `len` bytes into the U2TXREG FIFO. Spins on
     UTXBF when the FIFO is full. For our 10-byte response at 9600 baud
     the worst-case block is ~5 ms (8 bytes fit into the FIFO instantly;
     2 more wait for 1 byte each to drain at ~1 ms per byte). At a 5 Hz
     send rate that's well under our timing budget so not an issue
****************************************************************************/
static void TransmitFrame(const uint8_t *frame, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    while (U2STAbits.UTXBF)
    {
      // spin while TX FIFO is full
    }
    U2TXREG = frame[i];
  }
}

/****************************************************************************
 Function
     HandleDrivingMessage
 
 Description
     Process a Driving (status = 0x00) message from the paired MM. Drains
     steam appropriately, dispatches motion and digi commands to the
     ActuatorService (or all-stop if out of steam), and queues the response.
****************************************************************************/
static void HandleDrivingMessage(uint8_t joy1, uint8_t joy2, uint8_t digi)
{
  uint8_t cost = ComputeSteamCost(joy1, joy2, digi);
 
  if (Steam > 0) // do nothing if there is no steam
  {
    // Send the commands through to the actuator service.
    ES_Event_t evt;
 
    evt.EventType  = ES_SET_THRUSTERS;
    evt.EventParam = (uint16_t)(((uint16_t)joy1 << 8) | (uint16_t)joy2);
    PostActuatorService(evt);
 
    evt.EventType  = ES_SET_DIGI_OUT;
    evt.EventParam = (uint16_t)digi;
    PostActuatorService(evt);
 
    // Deduct steam, saturating at 0.
    if (cost >= Steam)
    {
      Steam = 0;
    }
    else
    {
      Steam -= cost;
    }
  } // end if steam not equal to 0
  else
  {
    // Out of steam: ignore commanded action, force everything idle.
    ES_Event_t evt;
    evt.EventType  = ES_ALL_STOP;
    evt.EventParam = 0;
    PostActuatorService(evt);
  } // end else if steam equal to 0
 
  // Tell the MM our current steam level.
  SendResponse(Steam);
}

/****************************************************************************
 Function
     HandleChargingMessage
 
 Description
     Add STEAM_CHARGE_INCREMENT to our steam total, cap at STEAM_MAX, and
     force everything idle (no propulsion while charging). Respond with the
     new steam level.
****************************************************************************/
static void HandleChargingMessage(void)
{
  uint16_t newSteam = (uint16_t)Steam + (uint16_t)STEAM_CHARGE_INCREMENT;
  if (newSteam > STEAM_MAX)
  {
    newSteam = STEAM_MAX;
  }
  Steam = (uint8_t)newSteam;
 
  // Charging means we're not driving; make sure motors and actuators idle.
  ES_Event_t evt;
  evt.EventType  = ES_ALL_STOP;
  evt.EventParam = 0;
  PostActuatorService(evt);
 
  SendResponse(Steam);
}

/****************************************************************************
 Function
     HandlePairingMessage
 
 Description
     handles Pairing message from a Mallard Module to which we're ALREADY paired.
     Per protocol: do not refill steam; respond with 0xFF in the charge byte.
****************************************************************************/
static void HandlePairingMessage(uint8_t pairingJoy1, uint8_t pairingJoy2)
{
  // No state change, no steam change; just the pair-ack response.
  SendResponse(CHARGE_BYTE_PAIR_ACK);
}

/****************************************************************************
 Function
     ComputeSteamCost
 
 Description
     Returns how many steam ticks this received Driving message should
     consume:
        0  -- idle (both joysticks centered, no actuation)
        1  -- propulsion only, OR dethroning only
        2  -- propulsion AND dethroning together (double rate per spec)
 * 
     The fauna gate (digi bit 0) does NOT drain steam per the project spec.

****************************************************************************/
static uint8_t ComputeSteamCost(uint8_t joy1, uint8_t joy2, uint8_t digi)
{
  int16_t diff1 = (int16_t)joy1 - (int16_t)JOY_CENTER;
  int16_t diff2 = (int16_t)joy2 - (int16_t)JOY_CENTER;
  //boolean that tracks if propulsion is active or not
  bool propActive = (diff1 >  JOY_DEADBAND) || (diff1 < -JOY_DEADBAND) ||
                    (diff2 >  JOY_DEADBAND) || (diff2 < -JOY_DEADBAND);
  // boolean that tracks if the ducksmacker is active
  bool dethroneActive = (digi & DIGI_BIT_SMACKER) != 0;
  // gate (digi bit 0) deliberately NOT counted
 
  if (propActive && dethroneActive) return 2;
  if (propActive || dethroneActive) return 1;
  return 0;
}

/****************************************************************************
 Function
     GoUnpaired
 
 Description
     Tear down the pairing: clear the stored MM address, stop motors and
     actuators, return the pairing indicator to "unpaired", and transition
     back to the Unpaired state. Cancel the pairing watchdog.
****************************************************************************/
static void GoUnpaired(void)
{
  PairedMM_AddrHi = 0x00;
  PairedMM_AddrLo = 0x00;
  Steam           = 0;
 
  // Make sure the boat is safe.
  ES_Event_t evt;
  evt.EventType  = ES_ALL_STOP;
  evt.EventParam = 0;
  PostActuatorService(evt);
 
  evt.EventType  = ES_SET_PAIR_IND;
  evt.EventParam = 0;
  PostActuatorService(evt);
 
  // Cancel the watchdog so it doesn't re-fire.
  ES_Timer_StopTimer(PAIRING_WATCHDOG_TIMER);
 
  CurrentState = QC_UNPAIRED;
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

