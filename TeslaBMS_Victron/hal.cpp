/*
  hal.cpp - hardware abstraction, see hal.h.
  Behaviour on the Teensy 3.2 is identical to the code that used to be in the sketch.
*/
#include "hal.h"

BmsCan bmsCan;

// ---------------------------------------------------------------- outputs (all models)
static uint8_t virtualOut[256 - PIN_VIRTUAL];

void halOutWrite(uint8_t pin, uint8_t value)
{
  if (pin >= PIN_VIRTUAL) { virtualOut[pin - PIN_VIRTUAL] = value ? 1 : 0; return; }
  digitalWrite(pin, value);
}

uint8_t halOutRead(uint8_t pin)
{
  if (pin >= PIN_VIRTUAL) return virtualOut[pin - PIN_VIRTUAL];
  return digitalRead(pin);
}

#if defined(BOARD_T3)
// ================================ Teensy 3.2 ================================
#include <FlexCAN.h>
// FlexCAN.h has compatibility macros "ext" -> flags.extended and "rtr" -> flags.remote;
// they would also rename BmsCanMsg::ext. Use the real field names here.
#undef ext
#undef rtr

void BmsCan::begin(uint32_t baud)
{
  Can0.begin(baud);
  CAN_filter_t allPassFilter; // enables extended addresses
  allPassFilter.id = 0;
  allPassFilter.flags.extended = 1;
  allPassFilter.flags.remote = 0;
  for (int filterNum = 4; filterNum < 16; filterNum++)
  {
    Can0.setFilter(allPassFilter, filterNum);
  }
}

void BmsCan::end()
{
  Can0.end();
}

bool BmsCan::read(BmsCanMsg &m)
{
  if (!Can0.available()) return false;
  CAN_message_t c;
  memset(&c, 0, sizeof(c));
  Can0.read(c);
  m.id = c.id;
  m.len = c.len;
  m.ext = c.flags.extended ? 1 : 0;
  memcpy(m.buf, c.buf, 8);
  return true;
}

bool BmsCan::write(const BmsCanMsg &m)
{
  CAN_message_t c;
  memset(&c, 0, sizeof(c));   // timeout 0, rtr 0 (as before: global message, zero-initialised)
  c.id = m.id;
  c.len = m.len;
  c.flags.extended = m.ext ? 1 : 0;
  memcpy(c.buf, m.buf, 8);
  return Can0.write(c) != 0;
}

uint32_t BmsCan::rxOverflows()
{
  return 0;
}

void halPrintResetReason(Stream &out)
{
  if (RCM_SRS1 & RCM_SRS1_SACKERR)   out.println("Stop Mode Acknowledge Error Reset");
  if (RCM_SRS1 & RCM_SRS1_MDM_AP)    out.println("MDM-AP Reset");
  if (RCM_SRS1 & RCM_SRS1_SW)        out.println("Software Reset");                   // reboot with SCB_AIRCR = 0x05FA0004
  if (RCM_SRS1 & RCM_SRS1_LOCKUP)    out.println("Core Lockup Event Reset");
  if (RCM_SRS0 & RCM_SRS0_POR)       out.println("Power-on Reset");                   // removed / applied power
  if (RCM_SRS0 & RCM_SRS0_PIN)       out.println("External Pin Reset");               // reboot with software download
  if (RCM_SRS0 & RCM_SRS0_WDOG)      out.println("Watchdog(COP) Reset");              // WDT timed out
  if (RCM_SRS0 & RCM_SRS0_LOC)       out.println("Loss of External Clock Reset");
  if (RCM_SRS0 & RCM_SRS0_LOL)       out.println("Loss of Lock in PLL Reset");
  if (RCM_SRS0 & RCM_SRS0_LVD)       out.println("Low-voltage Detect Reset");
}

void halWatchdogStart()
{
  noInterrupts();                                         // don't allow interrupts while setting up WDOG
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         // unlock access to WDOG registers
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1);                                   // need to wait a bit

  WDOG_TOVALH = 0x1000;
  WDOG_TOVALL = 0x0000;
  WDOG_PRESC  = 0;
  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
                  WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
                  WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  interrupts();
}

void halWatchdogKick()
{
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

static void (*lowVoltageCallback)() = nullptr;

static void lowVoltageIsr(void)
{
  if (lowVoltageCallback) lowVoltageCallback();
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK;  // clear if we can
  PMC_LVDSC1 |= PMC_LVDSC1_LVDACK;
}

void halLowVoltageStart(void (*callback)())
{
  lowVoltageCallback = callback;
  PMC_LVDSC1 =  PMC_LVDSC1_LVDV(1);  // enable hi v
  PMC_LVDSC2 = PMC_LVDSC2_LVWIE | PMC_LVDSC2_LVWV(3); // 2.92-3.08v
  attachInterruptVector(IRQ_LOW_VOLTAGE, lowVoltageIsr);
  NVIC_ENABLE_IRQ(IRQ_LOW_VOLTAGE);
}

#elif defined(BOARD_T4)
// ================================ Teensy 4.x ================================
#include <FlexCAN_T4.h>

static FlexCAN_T4<BMS_CAN_BUS, RX_SIZE_16, TX_SIZE_64> t4can;
static bool t4canStarted = false;

// Receive buffer, filled from the CAN interrupt (we do not use events(), so FlexCAN_T4
// calls the receive function directly in the interrupt). 256 messages: VE.Can carries
// ~400 frames/s and the module read round blocks the loop for ~0.7 s.
#define BMS_CAN_RXQ 256
static BmsCanMsg rxq[BMS_CAN_RXQ];
static volatile uint16_t rxHead = 0, rxTail = 0;
static volatile uint32_t rxLost = 0;

static void canRxIsr(const CAN_message_t &c)
{
  uint16_t next = (uint16_t)((rxHead + 1) % BMS_CAN_RXQ);
  if (next == rxTail) { rxLost++; return; }  // full: drop the newest
  BmsCanMsg &m = rxq[rxHead];
  m.id = c.id;
  m.len = c.len > 8 ? 8 : c.len;
  m.ext = c.flags.extended ? 1 : 0;
  memcpy(m.buf, c.buf, 8);
  rxHead = next;
}

void BmsCan::begin(uint32_t baud)
{
  if (!t4canStarted)
  {
    t4can.begin();
    t4can.setBaudRate(baud);
    t4can.setMaxMB(16);
    t4can.enableFIFO();
    t4can.setFIFOFilter(ACCEPT_ALL);   // standard and extended identifiers
    t4can.enableFIFOInterrupt();
    t4can.onReceive(canRxIsr);
    t4canStarted = true;
  }
  else
  {
    t4can.setBaudRate(baud);           // change of speed (menu)
  }
}

void BmsCan::end()
{
  // FlexCAN_T4 has no end(); begin() with a new speed is enough
}

bool BmsCan::read(BmsCanMsg &m)
{
  if (rxTail == rxHead) return false;
  noInterrupts();
  m = rxq[rxTail];
  rxTail = (uint16_t)((rxTail + 1) % BMS_CAN_RXQ);
  interrupts();
  return true;
}

bool BmsCan::write(const BmsCanMsg &m)
{
  CAN_message_t c;
  c.id = m.id;
  c.len = m.len;
  c.flags.extended = m.ext ? 1 : 0;
  memcpy(c.buf, m.buf, 8);
  return t4can.write(c) > 0;
}

uint32_t BmsCan::rxOverflows()
{
  return rxLost;
}

void halPrintResetReason(Stream &out)
{
  uint32_t s = SRC_SRSR;
  if (s & SRC_SRSR_LOCKUP_SYSRESETREQ) out.println("Software Reset / Core Lockup");
  if (s & SRC_SRSR_IPP_RESET_B)        out.println("Power-on Reset");
  if (s & SRC_SRSR_IPP_USER_RESET_B)   out.println("External Pin Reset");
  if (s & SRC_SRSR_WDOG_RST_B)         out.println("Watchdog (WDOG1) Reset");
  if (s & SRC_SRSR_WDOG3_RST_B)        out.println("Watchdog (WDOG3) Reset");
  if (s & SRC_SRSR_JTAG_SW_RST)        out.println("JTAG Software Reset");
  if (s & SRC_SRSR_TEMPSENSE_RST_B)    out.println("Temperature Sensor Reset");
  SRC_SRSR = s;                        // clear (write 1 to clear), so the next start shows its own reason
}

void halWatchdogStart()
{
  // WDOG3 (RTWDOG), 32 kHz LPO clock / 256 = 128 Hz, timeout 704 ticks = 5.5 s
  noInterrupts();
  CCM_CCGR5 |= CCM_CCGR5_WDOG3(CCM_CCGR_ON);
  WDOG3_CNT = 0xD928C520;                        // unlock
  while (!(WDOG3_CS & WDOG_CS_ULK)) ;
  WDOG3_TOVAL = 704;
  WDOG3_WIN = 0;
  WDOG3_CS = WDOG_CS_CMD32EN | WDOG_CS_CLK(1) | WDOG_CS_PRES | WDOG_CS_EN |
             WDOG_CS_UPDATE | WDOG_CS_WAIT | WDOG_CS_STOP;
  while (!(WDOG3_CS & WDOG_CS_RCS)) ;
  interrupts();
}

void halWatchdogKick()
{
  noInterrupts();
  WDOG3_CNT = 0xB480A602;                        // refresh
  interrupts();
}

void halLowVoltageStart(void (*callback)())
{
  (void)callback;  // no low voltage detection interrupt on the i.MX RT: SOC is saved as usual
}

#endif
