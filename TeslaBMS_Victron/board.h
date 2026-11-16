#pragma once
/*
  board.h - everything that differs per Teensy model / board.

  The rest of the firmware is the same for every Teensy. The compiler picks the
  block below from the processor it compiles for (Tools -> Board in the Arduino IDE,
  --fqbn in arduino-cli):
    Teensy 3.2  (MK20DX256)  -> SimpBMS V2 board (original hardware)
    Teensy 4.0  (IMXRT1062)  -> own board with CAN + Tesla serial bus
    Teensy 4.1  (IMXRT1062)  -> same as 4.0 (not built by GitHub, compiles)

  PIN_NONE (255) = not present on this board. digitalWrite/digitalRead/pinMode/
  analogWriteFrequency/attachInterrupt ignore it (digitalRead returns LOW).

  Own board with a different wiring? Change only the block of your Teensy here.
*/
#include <Arduino.h>

#define PIN_NONE 255

#if defined(__IMXRT1062__)
// ============================== Teensy 4.0 / 4.1 ==============================
  #if defined(ARDUINO_TEENSY41)
    #define BOARD_NAME "Teensy 4.1"
    #define BOARD_HW   "T41"
  #else
    #define BOARD_NAME "Teensy 4.0"
    #define BOARD_HW   "T40"
  #endif
  #define BOARD_T4 1

  // CAN (VE.Can / BMS-Can to the GX): FlexCAN_T4 bus
  //   CAN1 = pin 22 (TX) / 23 (RX)   CAN2 = pin 1 (TX) / 0 (RX)   CAN3 = pin 31 (TX) / 30 (RX)
  #define BMS_CAN_BUS CAN1

  // Tesla module bus (612500 baud). Serial2 = pin 7 (RX) / 8 (TX):
  // the same physical pins as Serial3 on the Teensy 3.2 / SimpBMS V2.
  #define SERIALBMS  Serial2

  // Display / serial CAN adapter (optional, 115200 baud). Serial5 = pin 21 (RX) / 20 (TX).
  #define SERIALDISP Serial5

  // Analogue current sensor (only for settings.cursens = analogue; not used with a Lynx/shunt via CAN)
  const int ACUR2 = A0; // pin 14
  const int ACUR1 = A1; // pin 15

  // Inputs / outputs (relays, contactors, PWM). Not present on this board: PIN_NONE.
  // Fill in the pin numbers if your board has them.
  const int IN1  = PIN_NONE; // input 1 - high active (key)
  const int IN2  = PIN_NONE; // input 2 - high active
  const int IN3  = PIN_NONE; // input 3 - high active (AC present)
  const int IN4  = PIN_NONE; // input 4 - high active (control pilot interrupt)
  const int OUT1 = PIN_NONE; // output 1 - high active (discharge)
  const int OUT2 = PIN_NONE; // output 2 - high active (contactor / breaker)
  const int OUT3 = PIN_NONE; // output 3 - high active (charger)
  const int OUT4 = PIN_NONE; // output 4 - high active (precharge)
  const int OUT5 = PIN_NONE; // output 5 - low active PWM
  const int OUT6 = PIN_NONE; // output 6 - low active PWM
  const int OUT7 = PIN_NONE; // output 7 - low active PWM
  const int OUT8 = PIN_NONE; // output 8 - low active PWM
  const int led  = 13;

#elif defined(__MK20DX256__)
// ============================== Teensy 3.2 on SimpBMS V2 ==============================
  #define BOARD_NAME "Teensy 3.2 (SimpBMS V2)"
  #define BOARD_HW   "T32"
  #define BOARD_T3 1

  // CAN: FlexCAN (Can0), pin 3 (TX) / 4 (RX), transceiver on the SimpBMS V2 board
  // Tesla module bus: Serial3 = pin 7 (RX) / 8 (TX)
  #define SERIALBMS  Serial3
  // Display / serial CAN adapter: Serial2 = pin 9 (RX) / 10 (TX)
  #define SERIALDISP Serial2

  const int ACUR2 = A0; // current 1
  const int ACUR1 = A1; // current 2
  const int IN1  = 17; // input 1 - high active
  const int IN2  = 16; // input 2 - high active
  const int IN3  = 18; // input 3 - high active
  const int IN4  = 19; // input 4 - high active
  const int OUT1 = 11; // output 1 - high active
  const int OUT2 = 12; // output 2 - high active
  const int OUT3 = 20; // output 3 - high active
  const int OUT4 = 21; // output 4 - high active
  const int OUT5 = 22; // output 5 - low active
  const int OUT6 = 23; // output 6 - low active
  const int OUT7 = 5;  // output 7 - low active
  const int OUT8 = 6;  // output 8 - low active
  const int led  = 13;

#else
  #error "Unsupported board: select Teensy 3.2 or Teensy 4.0/4.1 (see board.h)"
#endif
