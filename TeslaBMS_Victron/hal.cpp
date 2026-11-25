/*
  hal.cpp - hardware abstraction, see hal.h.
  Behaviour on the Teensy 3.2 is identical to the code that used to be in the sketch.
*/
#include "hal.h"

BmsCan bmsCan;

// ---------------------------------------------------------------- CAN bus health (all models)
#if defined(BOARD_T4)
  #define BMS_CAN_BASE BMS_CAN_REG_BASE        // board.h
#else
  #define BMS_CAN_BASE 0x40024000UL      // FlexCAN0 of the MK20DX256 (Teensy 3.2)
#endif
#define BMS_CAN_ECR  (*(volatile uint32_t *)(BMS_CAN_BASE + 0x1C))
#define BMS_CAN_ESR1 (*(volatile uint32_t *)(BMS_CAN_BASE + 0x20))
static volatile uint32_t canTxOk = 0, canTxFail = 0, canRx = 0;
static uint16_t canErrFlags = 0;
static uint32_t canErrEvents = 0;
static uint32_t canLastEsr = 0;

void BmsCan::poll()
{
  uint32_t esr = BMS_CAN_ESR1;          // reading clears the error bits
  canLastEsr = esr;
  uint16_t e = (uint16_t)(esr & 0xFF00);
  if (e & 0xFC00) canErrEvents++;
  canErrFlags |= e;
}

void BmsCan::stats(BmsCanStats &s, bool resetFlags)
{
  poll();
  uint32_t ecr = BMS_CAN_ECR;
  s.txOk = canTxOk; s.txFail = canTxFail; s.rx = canRx; s.rxLost = rxOverflows();
  s.tec = ecr & 0xFF; s.rec = (ecr >> 8) & 0xFF;
  uint8_t flt = (canLastEsr >> 4) & 0x3;
  s.state = (flt == 0) ? 0 : (flt == 1 ? 1 : 2);
  s.errFlags = canErrFlags; s.errEvents = canErrEvents;
  if (resetFlags) { canErrFlags = 0; canErrEvents = 0; }
}

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
  canRx++;
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
  bool ok = Can0.write(c) != 0;
  if (ok) canTxOk++; else canTxFail++;
  return ok;
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

// NO interrupts (polling): with the FIFO/mailbox interrupts of FlexCAN_T4 enabled, the Teensy 4.0
// stopped running (no output at all) as soon as it was on a bus where other devices acknowledge
// its messages (SolarEdge + GX), while it ran fine without a bus. Polling is simple and robust;
// the receive FIFO holds 6 messages, so during the ~0.7 s module read round some received
// messages can be lost (fine for the inverter protocols; the shunt current comes over USB).
static volatile uint32_t rxLost = 0;

void BmsCan::begin(uint32_t baud)
{
  if (!t4canStarted)
  {
    t4can.begin();
    t4can.setBaudRate(baud);
    t4can.setMaxMB(16);
    t4can.enableFIFO();
    t4can.setFIFOFilter(ACCEPT_ALL);   // standard and extended identifiers
    t4can.enableFIFOInterrupt(false);   // polling, see above
    t4can.enableMBInterrupts(false);    // also no transmit-mailbox interrupts
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
  CAN_message_t c;
  if (!t4can.read(c)) return false;
  m.id = c.id;
  m.len = c.len > 8 ? 8 : c.len;
  m.ext = c.flags.extended ? 1 : 0;
  memcpy(m.buf, c.buf, 8);
  canRx++;
  return true;
}

bool BmsCan::write(const BmsCanMsg &m)
{
  CAN_message_t c;
  c.id = m.id;
  c.len = m.len;
  c.flags.extended = m.ext ? 1 : 0;
  memcpy(c.buf, m.buf, 8);
  bool ok = t4can.write(c) > 0;
  if (ok) canTxOk++; else canTxFail++;
  return ok;
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
