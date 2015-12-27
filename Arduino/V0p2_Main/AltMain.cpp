/*
The OpenTRV project licenses this file to you
under the Apache Licence, Version 2.0 (the "Licence");
you may not use this file except in compliance
with the Licence. You may obtain a copy of the Licence at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing,
software distributed under the Licence is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
KIND, either express or implied. See the Licence for the
specific language governing permissions and limitations
under the Licence.

Author(s) / Copyright (s): Damon Hart-Davis 2014--2015
*/

/*
  Alternate POST / setup and loop / main for non-OpenTRV code running on OpenTRV h/w platform.

  DHD20151208: TEST CODE FOR REV7 PRODUCTION 2015Q4.
 */


#include "V0p2_Main.h"

#include "V0p2_Generic_Config.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

// Arduino libraries.
//#include <Wire.h>
#ifdef ALLOW_CC1_SUPPORT
#include <OTProtocolCC.h>
#endif
#include <OTV0p2Base.h>
#include <OTRadioLink.h>

#include "Control.h"
#include "Power_Management.h"
#include "Radio.h"
#include "Serial_IO.h"
#include "UI_Minimal.h"
#include "V0p2_Sensors.h"

#include <avr/pgmspace.h> // for radio config




// Link in support for alternate Power On Self-Test and main loop if required.
#if defined(ALT_MAIN_LOOP)

// Mask for Port B input change interrupts.
#define MASK_PB_BASIC 0b00000000 // Nothing.
#ifdef PIN_RFM_NIRQ
  #if (PIN_RFM_NIRQ < 8) || (PIN_RFM_NIRQ > 15)
    #error PIN_RFM_NIRQ expected to be on port B
  #endif
  #define RFM23B_INT_MASK (1 << (PIN_RFM_NIRQ&7))
  #define MASK_PB (MASK_PB_BASIC | RFM23B_INT_MASK)
#else
  #define MASK_PB MASK_PB_BASIC
#endif


// Mask for Port D input change interrupts.
#define MASK_PD_BASIC 0b00000001 // Just RX.
#if defined(ENABLE_VOICE_SENSOR)
#if VOICE_NIRQ > 7
#error voice interrupt on wrong port
#endif
#define VOICE_INT_MASK (1 << (VOICE_NIRQ&7))
#define MASK_PD (MASK_PD_BASIC | VOICE_INT_MASK)
#else
#define MASK_PD MASK_PD_BASIC // Just RX.
#endif


// Called from startup() after some initial setup has been done.
// Can abort with panic() if need be.
void POSTalt()
  {

//#if defined(USE_MODULE_RFM22RADIOSIMPLE) 
//  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
//  PrimaryRadio.preinit(NULL);
//  // Check that the radio is correctly connected; panic if not...
//  if(!PrimaryRadio.configure(1, &RFMConfig) || !PrimaryRadio.begin()) { panic(F("PANIC!")); }
//#endif


  // Force initialisation into low-power state.
  const int heat = TemperatureC16.read();
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp: ");
  DEBUG_SERIAL_PRINT(heat);
  DEBUG_SERIAL_PRINTLN();
#endif
  const int light = AmbLight.read();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("light: ");
  DEBUG_SERIAL_PRINT(light);
  DEBUG_SERIAL_PRINTLN();
#endif



  // Trailing setup for the run
  // --------------------------

  // Set up async edge interrupts.
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    //PCMSK0 = PB; PCINT  0--7    (LEARN1 and Radio)
    //PCMSK1 = PC; PCINT  8--15
    //PCMSK2 = PD; PCINT 16--24   (LEARN2 and MODE, RX)

    PCICR =
#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
        1 | // 0x1 enables PB/PCMSK0.
#endif
#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
        2 | // 0x2 enables PC/PCMSK1.
#endif
#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
        4 | // 0x4 enables PD/PCMSK2.
#endif
        0;

#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
    PCMSK0 = MASK_PB;
#endif
#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
    PCMSK1 = MASK_PC;
#endif
#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
    PCMSK2 = MASK_PD;
#endif
    }


//  pinMode(3, INPUT);        // FIXME Move to where they are set automatically
//  digitalWrite(3, LOW);

  bareStatsTX(false, false, false);

  }


#if defined(ALT_MAIN_LOOP)

#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
//// Interrupt count.  Marked volatile so safe to read without a lock as is a single byte.
//static volatile uint8_t intCountPB;
// Previous state of port B pins to help detect changes.
static volatile uint8_t prevStatePB;
// Interrupt service routine for PB I/O port transition changes.
ISR(PCINT0_vect)
  {
//  ++intCountPB;
  const uint8_t pins = PINB;
  const uint8_t changes = pins ^ prevStatePB;
  prevStatePB = pins;

#if defined(RFM23B_INT_MASK)
  // RFM23B nIRQ falling edge is of interest.
  // Handler routine not required/expected to 'clear' this interrupt.
  // TODO: try to ensure that OTRFM23BLink.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
  if((changes & RFM23B_INT_MASK) && !(pins & RFM23B_INT_MASK))
    { PrimaryRadio.handleInterruptSimple(); }
#endif

  }
#endif

#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
// Previous state of port C pins to help detect changes.
static volatile uint8_t prevStatePC;
// Interrupt service routine for PC I/O port transition changes.
ISR(PCINT1_vect)
  {
//  const uint8_t pins = PINC;
//  const uint8_t changes = pins ^ prevStatePC;
//  prevStatePC = pins;
//
// ...
  }
#endif

#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
// Previous state of port D pins to help detect changes.
static volatile uint8_t prevStatePD;
// Interrupt service routine for PD I/O port transition changes (including RX).
ISR(PCINT2_vect)
  {

  const uint8_t pins = PIND;
  const uint8_t changes = pins ^ prevStatePD;
  prevStatePD = pins;

#if defined(ENABLE_VOICE_SENSOR)
  //  // Voice detection is a falling edge.
  //  // Handler routine not required/expected to 'clear' this interrupt.
  //  // FIXME: ensure that Voice.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
    // Voice detection is a RISING edge.
    if((changes & VOICE_INT_MASK) && (pins & VOICE_INT_MASK)) {
      Voice.handleInterruptSimple();
    }
#endif // ENABLE_VOICE_SENSOR

//    // If an interrupt arrived from no other masked source then wake the CLI.
//    // The will ensure that the CLI is active, eg from RX activity,
//    // eg it is possible to wake the CLI subsystem with an extra CR or LF.
//    // It is OK to trigger this from other things such as button presses.
//    // FIXME: ensure that resetCLIActiveTimer() is inlineable to minimise ISR prologue/epilogue time and space.
//    if(!(changes & MASK_PD & ~1)) { resetCLIActiveTimer(); }
  }
#endif // defined(MASK_PD) && (MASK_PD != 0)

#endif // ALT_MAIN







// Called from loop().
void loopAlt()
  {
  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*E"); // End-of-cycle sleep.
#endif

#if !defined(MIN_ENERGY_BOOT)
  OTV0P2BASE::powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  minimisePowerWithoutSleep();
#endif
  static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = OTV0P2BASE::getSecondsLT()))
    {
    // Poll I/O and process message incrementally (in this otherwise idle time)
    // before sleep and on wakeup in case some IO needs further processing now,
    // eg work was accrued during the previous major slow/outer loop
    // or the in a previous orbit of this loop sleep or nap was terminated by an I/O interrupt.
    // Come back and have another go if work was done, until the next tick at most.
    if(handleQueuedMessages(&Serial, true, &PrimaryRadio)) { continue; }

// If missing h/w interrupts for anything that needs rapid response
// then AVOID the lowest-power long sleep.
#if CONFIG_IMPLIES_MAY_NEED_CONTINUOUS_RX && !defined(PIN_RFM_NIRQ)
#define MUST_POLL_FREQUENTLY true
#else
#define MUST_POLL_FREQUENTLY false
#endif
    if(MUST_POLL_FREQUENTLY /** && in hub mode */ )
      {
      // No h/w interrupt wakeup on receipt of frame,
      // so can only sleep for a short time between explicit poll()s,
      // though allow wake on interrupt anyway to minimise loop timing jitter.
      OTV0P2BASE::nap(WDTO_15MS, true);
      }
    else
      {
      // Normal long minimal-power sleep until wake-up interrupt.
      // Rely on interrupt to force fall through to I/O poll() below.
      OTV0P2BASE::sleepUntilInt();
      }
//    DEBUG_SERIAL_PRINTLN_FLASHSTRING("w"); // Wakeup.

//    idle15AndPoll(); // Attempt to crash the board!

    }
  TIME_LSD = newTLSD;

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif


  // START LOOP BODY
  // ===============

//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*");

//  // Power up serial for the loop body.
//  // May just want to turn it on in POSTalt() and leave it on...
//  const bool neededWaking = powerUpSerialIfDisabled();







   const uint8_t l = AmbLight.read();
   DEBUG_SERIAL_PRINT(l);
   DEBUG_SERIAL_PRINTLN();







//  // Force any pending output before return / possible UART power-down.
//  flushSerialSCTSensitive();
//  if(neededWaking) { OTV0P2BASE::powerDownSerial(); }
  }



#endif
