#pragma once
/*
  hal.h - hardware abstraction: the few things that are done differently on the
  Teensy 3.2 (Kinetis MK20DX256) and the Teensy 4.x (i.MX RT1062).
  The implementation is in hal.cpp; the pins and ports are in board.h.

  - CAN:        FlexCAN (Teensy 3.2) or FlexCAN_T4 (Teensy 4.x), one message type BmsCanMsg
  - watchdog:   WDOG (3.2) or WDOG3 (4.x), about 5.5 s
  - reset reason, low supply voltage detection (3.2 only)
*/
#include <Arduino.h>
#include "board.h"

struct BmsCanMsg
{
  uint32_t id;
  uint8_t  len;
  uint8_t  buf[8];
  uint8_t  ext;   // 1 = 29-bit identifier
};

class BmsCan
{
public:
  void begin(uint32_t baud);          // also used to change the speed
  void end();
  bool read(BmsCanMsg &m);            // true = a message was read
  bool write(const BmsCanMsg &m);     // true = accepted for sending
  uint32_t rxOverflows();             // messages lost because the receive buffer was full (4.x)
};

extern BmsCan bmsCan;

// Outputs OUT1..OUT8: write / read back. Real pins: digitalWrite / digitalRead.
// Virtual outputs (PIN_VIRTUAL + n, not present on the board): only the value is remembered.
void halOutWrite(uint8_t pin, uint8_t value);
uint8_t halOutRead(uint8_t pin);

void halPrintResetReason(Stream &out);
void halWatchdogStart();               // about 5.5 s
void halWatchdogKick();
void halLowVoltageStart(void (*callback)()); // callback on low supply voltage (3.2 only; no-op on 4.x)
