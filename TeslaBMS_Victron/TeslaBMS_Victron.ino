/*
  Copyright (c) 2019 Simp ECO Engineering
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
  ==========================================================================
  TeslaBMS_Victron - based on TeslaBMSV2 (SimpBMS, firmver 230719)
  Adapted for Tesla 6S modules on Victron VenusOS (DVCC / CAN-bus BMS).

  Runs on Teensy 3.2 (SimpBMS V2) and Teensy 4.0/4.1 with the same code:
  pins and serial ports per model in board.h, CAN/watchdog/reset reason in hal.cpp.

  Main changes compared to TeslaBMSV2:
  - Fixed topology instead of "detect whatever happens to be found":
      Scells   = 6 (24 V, 1 module per string) or 12 (48 V, 2 modules in series)
      Pstrings = number of strings in parallel
    Address 1..M = string 1, M+1..2M = string 2, ... (M = Scells/6).
    The communication chain must be wired in that order.
  - Robust module detection (CRC check, buffer flush, no renumbering
    at every start). Renumber manually: menu 'm' -> 'r'.
  - Missing module/string: no Error, CVL/CCL/DCL keep being sent.
      * immediately: warning "BMS internal" in 0x35A
      * after ModuleAlarmDelay seconds: alarm "BMS internal" in 0x35A
      * 0x372: modules online / offline (physical number of Tesla modules)
      * CCL/DCL scaled with strings online (setting ScaleLimits)
      * every 30 s an automatic search for the missing modules
  - 0x355 SOC high resolution (0.01%) fixed (was a factor 10 too low).
  - ESS mode: over/undervoltage delay timers fixed.
  - ESS mode: cell voltage never gives Error (Error sets CCL and DCL to 0). Normal range
    3.00 - 4.10 V. Too high (>= OverVSetpoint 4.20 V): CCL = 0, DCL normal. Too low
    (DischVsetpoint 3.00 V): DCL = 0, charging allowed; below UnderVSetpoint (2.90 V) slowly
    (chargecurrentend per string), never CCL = 0. No contactor/breaker trip for cell voltage.
  ==========================================================================
*/

#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <ADC.h> //https://github.com/pedvide/ADC
#include <EEPROM.h>
#include "hal.h"      // CAN, watchdog, reset reason per Teensy model (board.h = pins)
#include <SPI.h>
#include "version.h"     // firmware version (FW_YEAR.FW_BUILD), raised with every commit
#include "LowPassFilter.h" // built-in low-pass filter (replaces the external Filters library)
#include "Serial_CAN_Module_TeensyS2.h" //https://github.com/tomdebree/Serial_CAN_Teensy

/*
  #define CPU_REBOOT (_reboot_Teensyduino_());
*/
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define CPU_REBOOT WRITE_RESTART(0x5FA0004)


Serial_CAN can;
BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;

/////Version Identifier/////////
int firmver = 261104; //Year Month Day (TeslaBMS_Victron)

//////// NEW: module/string monitoring ////////
bool moduleWarning = false;          // fewer strings online than set
bool moduleAlarm = false;            // ... for longer than settings.ModuleAlarmDelay
unsigned long moduleMissingSince = 0;
unsigned long rescanTimer = 0;
#define RESCAN_INTERVAL 30000        // ms between searches for missing modules

//////// NEW: Victron-style CVL control ////////
#define CVL_DOWN_RATE 0.10f   // V per second lowering as long as the highest cell is above the charge voltage
#define CVL_UP_RATE   0.01f   // V per second raising again when there is room (slow, prevents hunting)
#define CVL_BAND      0.010f  // V per cell: raise only when the highest cell is this far below the target
#define CVL_FAR_BAND  0.050f  // V per cell: highest cell this far below the target -> CVL straight to the maximum
#define CVL_START_MARGIN 0.10f // V: when lowering, start no higher than battery voltage + this margin
#define OV_RELEASE    0.050f  // V per cell: CCL=0 due to overvoltage is released below OverV - this value
#define OV_ALARM_MARGIN 0.050f // V per cell: high voltage ALARM at OverV + this value (default 4.25 V),
                               // low voltage ALARM at UnderV - this value (default 2.85 V);
                               // the WARNINGS are at OverV (4.20 V) and UnderV (2.90 V) themselves
#define LG_MIN_PACK_V 41.0f   // V: LG RESU / SolarEdge RWS mode: the inverter accepts 40-62 V; DCL = 0 at least here

// NEW: log of received CAN identifiers (for a new inverter: which messages does it send?)
#define CANLOG_SIZE 16
struct CanLogEntry { uint32_t id; uint8_t ext; uint8_t len; uint32_t count; uint8_t buf[8]; };
CanLogEntry canLog[CANLOG_SIZE];
uint8_t canLogUsed = 0;
uint32_t lgKeepAliveMs = 0;   // last 0x305 'keep alive' from the inverter (LG RESU protocol)
uint32_t lastCurrentMs = 0;   // last current measurement received over CAN (Lynx / shunt)
bool socFromVoltage = false;  // SOC calculated from the cell voltage (no current measurement)
float extSoc = -1.0f;         // SOC from outside over USB ('S' command, e.g. Lynx shunt on the GX via Node-RED)
uint32_t extSocMs = 0;        // when it was received
bool socExternal = false;     // SOC taken from extSoc

void canLogAdd(const BmsCanMsg &m)
{
  for (uint8_t i = 0; i < canLogUsed; i++)
  {
    if (canLog[i].id == m.id && canLog[i].ext == m.ext)
    {
      canLog[i].count++; canLog[i].len = m.len; memcpy(canLog[i].buf, m.buf, 8);
      return;
    }
  }
  if (canLogUsed < CANLOG_SIZE)
  {
    CanLogEntry &e = canLog[canLogUsed++];
    e.id = m.id; e.ext = m.ext; e.len = m.len; e.count = 1; memcpy(e.buf, m.buf, 8);
  }
}

// Lower end of the normal range (DCL = 0) for the selected inverter
float dischargeCellV()
{
  float v = settings.DischVsetpoint;
  if (settings.InverterProto == 1 && settings.Scells > 0 && v * settings.Scells < LG_MIN_PACK_V)
    v = LG_MIN_PACK_V / settings.Scells;   // SolarEdge RWS switches off below 40 V
  return v;
}

float cvlActual = 0.0f;       // V, the CVL sent to Victron
unsigned long cvlTimer = 0;
bool overVoltLatch = false;   // highest cell >= OverVSetpoint: CCL = 0 (chargers off)
bool eepromWasReset = false;  // set at the one-time EEPROM reset (message at startup)
unsigned long jsonTimer = 0;
#define JSON_INTERVAL 5000UL  // ms between two JSON lines (Node-RED dashboard)
unsigned long lastMenuInput = 0;
unsigned long lastBlockMs = 0; // ms: duration of the last read round (500 ms block)
#define MENU_TIMEOUT 120000UL // ms: close the menu automatically without input

//Curent filter//
float filterFrequency = 5.0 ;
LowPassFilter lowpassFilter( filterFrequency );

//Simple BMS V2 wiring//
// Pins (IN1..4, OUT1..8, ACUR1/2, led) and serial ports: see board.h
const int BMBfault = 11;

byte bmsstatus = 0;
//bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define Error 5
//
//Current sensor values
#define Undefined 0
#define Analoguedual 1
#define Canbus 2
#define Analoguesing 3

// Can current sensor values
#define LemCAB300 1
#define IsaScale 3
#define VictronLynx 4
#define LemCAB500 2
#define CurCanMax 4 // max value

//
//Charger Types
#define NoCharger 0
#define BrusaNLG5 1
#define ChevyVolt 2
#define Eltek 3
#define Elcon 4
#define Victron 5
#define Coda 6
#define VictronHV 7
//



int Discharge;

//variables for output control
int pulltime = 100;
int contctrl, contstat = 0; //1 = out 5 high 2 = out 6 high 3 = both high
unsigned long conttimer1, conttimer2, conttimer3, Pretimer, Pretimer1, overtriptimer, undertriptimer, mainconttimer = 0;
uint16_t pwmfreq = 18000;//pwm frequency
int pwmcurmax = 50;//Max current to be shown with pwm
int pwmcurmid = 50;//Mid point for pwm dutycycle based on current
int16_t pwmcurmin = 0;//DONOT fill in, calculated later based on other values

bool OutputEnable = 0;
bool CanOnReq  = false;
bool CanOnRev = false;

//variables for VE can
uint16_t chargevoltage = 49100; //max charge voltage in mv
int32_t  chargecurrent = 0, tempchargecurrent = 0; // 0.1A, TOTAL for the battery. Signed: see currentlimit()
uint16_t disvoltage = 42000; // max discharge voltage in mv
int32_t  discurrent = 0; // 0.1A, TOTAL for the battery
int batvcal = 0;

uint16_t SOH = 100; // SOH place holder

unsigned char alarm[4] = {0, 0, 0, 0};
unsigned char warning[4] = {0, 0, 0, 0};
unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// Names towards VenusOS (each CAN field is max. 8 characters, padded with spaces)
// 0x35E must be EQUAL to the first 8 characters of 0x370 (CAN-bus BMS protocol) -> "Claude T"
unsigned char bmsname[8] = {'C', 'l', 'a', 'u', 'd', 'e', ' ', 'T'};        // 0x35E
const char batteryName[17] = "Claude Tesla BMS";                              // 0x370 + 0x371 battery name (16 characters)
long unsigned int rxId;
unsigned char len = 0;
byte rxBuf[8];
char msgString[128];                        // Array to store serial string
uint32_t inbox;
signed long CANmilliamps;//mV
signed long voltage1, voltage2, voltage3 = 0; //mV only with ISAscale sensor
//struct can_frame canMsg;
//MCP2515 CAN1(10); //set CS pin for can controlelr


//variables for current calulation
int value;
float currentact, RawCur;
float ampsecond;
unsigned long lasttime;
unsigned long looptime, looptime1, UnderTimer, OverTime, cleartime, baltimer, CanOntimeout = 0; //ms
int currentsense = 14;
int sensor = 1;

//Variables for SOC calc
int SOC = 100; //State of Charge
int SOCset = 0;
int SOCtest = 0;
int SOCmem = 0;
int SOCreset = 0;

///charger variables
int maxac1 = 16; //Shore power 16A per charger
int maxac2 = 10; //Generator Charging
int chargerid1 = 0x618; //bulk chargers
int chargerid2 = 0x638; //finishing charger
float chargerendbulk = 0; //V before Charge Voltage to turn off the bulk charger/s
float chargerend = 0; //V before Charge Voltage to turn off the finishing charger/s
int chargertoggle = 0;
int ncharger = 1; // number of chargers
bool chargecurrentlimit = 0;

//serial canbus expansion
unsigned long id = 0;
unsigned char dta[8];

//AC current control
volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh, duration = 0;
volatile uint16_t accurlim = 0;
volatile int dutycycle = 0;
uint16_t chargerpower = 0;
bool CPdebug = 0;

//variables
int outputstate = 0;
int incomingByte = 0;
int x = 0;
int storagemode = 0;
int cellspresent = 0;
int dashused = 1;
int Charged = 0;
int renum = 0;

//Debugging modes//////////////////
int debug = 1;
int inputcheck = 0; //read digital inputs
int outputcheck = 0; //check outputs
int candebug = 0; //view can frames
int gaugedebug = 0;
int debugCur = 0;
int CSVdebug = 0;
int delim = 0;
int menuload = 0;
int balancecells;
int debugdigits = 2; //amount of digits behind decimal for voltage reading

int testcount = 0;

ADC *adc = new ADC(); // adc object

void loadSettings()
{
  Logger::console("Resetting to factory defaults");
  settings.version = EEPROM_VERSION;
  settings.checksum = 2;
  settings.canSpeed = 500000;
  settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.2f;
  settings.UnderVSetpoint = 2.9f;  // below this: warning + slow charging (normal range ends at DischVsetpoint)
  settings.ChargeVsetpoint = 4.1f;
  settings.ChargeHys = 0.2f; // voltage drop required for charger to kick back on
  settings.WarnOff = 0.1f; // not used anymore (warnings at OverVSetpoint / UnderVSetpoint)
  settings.DischVsetpoint = 3.0f;  // DCL = 0 here: lower end of the normal range 3.00 - 4.10 V
  settings.DischHys = 0.2f; // Discharge voltage offset
  settings.CellGap = 0.2f; //max delta between high and low cell
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.ChargeTSetpoint = 0.0f;
  settings.triptime = 500;//mS of delay before counting over or undervoltage
  settings.DisTSetpoint = 40.0f;
  settings.WarnToff = 5.0f; //temp offset before raising warning
  settings.IgnoreTemp = 0; // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 1.0f; // cell readings below this (and above 4.5 V) are sensor faults and are ignored
  settings.balanceVoltage = 3.9f;
  settings.balanceHyst = 0.005f; // balance down to 5 mV difference to the lowest cell in the string
  settings.balanceDuty = 50;
  settings.logLevel = 2;
  settings.CAP = 230; //Ah PER STRING (Tesla 6S74P module approx. 230Ah - check your type)
  settings.Pstrings = 1; // number of strings in parallel
  settings.Scells = 12;//Cells in series per string: 6 = 24V (1 Tesla module), 12 = 48V (2 modules); default 48 V
  settings.StoreVsetpoint = 3.8; // V storage mode charge max
  settings.discurrentmax = 250; // max discharge current in 0.1A PER STRING (25 A)
  settings.DisTaper = 0.3f; //V offset to bring in discharge taper to Zero Amps at settings.DischVsetpoint
  settings.chargecurrentmax = 250; //max charge current in 0.1A PER STRING (25 A)
  settings.chargecurrent2max = 125; //alternative max charge current in 0.1A PER STRING (12.5 A)
  settings.chargecurrentend = 25; //end charge current in 0.1A PER STRING (2.5 A)
  settings.PulseCh = 600; //Peak Charge current in 0.1A
  settings.PulseChDur = 5000; //Ms of discharge pulse derating
  settings.PulseDi = 600; //Peak Charge current in 0.1A
  settings.PulseDiDur = 5000; //Ms of discharge pulse derating
  settings.socvolt[0] = 3100; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[1] = 10; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[2] = 4100; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[3] = 90; //Voltage and SOC curve for voltage based SOC calc
  settings.invertcur = 0; //Invert current sensor direction
  settings.cursens = Canbus;        // current via CAN
  settings.curcan = VictronLynx;    // Victron Lynx/shunt (PGN 127508 on VE.Can)
  settings.voltsoc = 0; //SOC purely voltage based
  settings.Pretime = 5000; //ms of precharge time
  settings.conthold = 50; //holding duty cycle for contactor 0-255
  settings.Precurrent = 1000; //ma before closing main contator
  settings.convhigh = 580; // mV/A current sensor high range channel
  settings.convlow = 6430; // mV/A current sensor low range channel
  settings.offset1 = 1750; //mV mid point of channel 1
  settings.offset2 = 1750;//mV mid point of channel 2
  settings.changecur = 20000;//mA change overpoint
  settings.gaugelow = 50; //empty fuel gauge pwm
  settings.gaugehigh = 255; //full fuel gauge pwm
  settings.ESSmode = 1; //ESS mode (stationary storage): always available, recovers by itself after an error
  settings.ncur = 1; //number of multiples to use for current measurement
  settings.chargertype = NoCharger; // Victron controls charging via DVCC (CVL/CCL); no car charger messages on CAN
  settings.ChargerDirect = 1;       // ESS: no enable input (IN2) needed
  settings.chargerspd = 100; //ms per message
  settings.chargereff = 85; //% effiecency of charger
  settings.chargerACv = 240;// AC input voltage into Charger
  settings.UnderDur = 5000; //ms of allowed undervoltage before throwing open stopping discharge.
  settings.CurDead = 5;// mV of dead band on current sensor
  settings.ExpMess = 0; //send alternate victron info
  settings.SerialCan = 0; //Serial canbus or display: 0-display 1- canbus expansion
  settings.tripcont = 1; //in ESSmode 1 - Main contactor function, 0 - Trip function
  loadNewSettings();
}

// NEW: defaults for the settings added in TeslaBMS_Victron
void loadNewSettings()
{
  settings.ModuleAlarmDelay = 60; // s missing before warning becomes alarm
  settings.ScaleLimits = 1;       // scale CCL/DCL with strings online
  settings.balanceStart = 0.015f; // start balancing at > 15 mV difference
  settings.AutoTopology = 1;      // at every start: count modules = expected number
  settings.CVLControl = 1;        // Victron style: lower CVL, CCL stays open until OverVSetpoint
  settings.StorageInput = 0;      // storage mode via IN1 off: a full battery stays at ChargeVsetpoint
  settings.InverterProto = 0;     // CAN protocol: 0 = Victron, 1 = LG RESU LV (SolarEdge StorEdge RWS)
  settings.JsonOut = 0;           // JSON output over USB (Node-RED) off by default
}

/*
  NEW: auto-detect at startup (settings.AutoTopology = 1).
  Renumbers all modules in chain order and takes the number found as the
  expected number of strings: strings = found modules / modules per string (rounded up,
  so an incomplete 48V string becomes visible as offline).
  If a module drops out AFTER that, the normal warning/alarm still follows.
*/
void autoDetectTopology()
{
  bms.renumberBoardIDs();
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
  applyTopology();
}

/*
  NEW: apply the topology after a change of Scells (or at startup).
  With auto-detect ON the number of strings follows from the modules found:
  strings = found modules / (Scells/6), rounded up. So switching from 24 V (6S) to
  48 V (12S) only requires changing Scells: 6 modules -> 6 strings becomes 3 strings.
  The CVL control is also restarted (new maximum CVL).
*/
void applyTopology()
{
  if (settings.Scells < 6) settings.Scells = 6;
  settings.Scells = (settings.Scells / 6) * 6;   // Tesla module = 6 cells
  int mps = settings.Scells / 6;
  if (settings.AutoTopology == 1)
  {
    int found = bms.getNumModules();
    int strings = (found + mps - 1) / mps;
    if (strings < 1) strings = 1;
    settings.Pstrings = strings;
    SERIALCONSOLE.println();
    SERIALCONSOLE.print("Auto-detect: ");
    SERIALCONSOLE.print(found);
    SERIALCONSOLE.print(" module(s), ");
    SERIALCONSOLE.print(mps);
    SERIALCONSOLE.print(" per string -> ");
    SERIALCONSOLE.print(strings);
    SERIALCONSOLE.println(" string(s)");
  }
  bms.setTopology(settings.Scells, settings.Pstrings);
  bms.updateStringStatus();
  cvlActual = 0.0f;   // CVL control starts again from the new maximum value
  bms.printTopology();
}

BmsCanMsg msg;     // outgoing CAN message (zero-initialised, fields are kept between messages)
BmsCanMsg inMsg;   // incoming CAN message

uint32_t lastUpdate;


void setup()
{
  SERIALBMS.begin(612500); //Tesla serial bus
  delay(2000);  //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's
  //pinMode(ACUR1, INPUT);//Not required for Analogue Pins
  //pinMode(ACUR2, INPUT);//Not required for Analogue Pins
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(IN4, INPUT);
  pinMode(OUT1, OUTPUT); // drive contactor
  pinMode(OUT2, OUTPUT); // precharge
  pinMode(OUT3, OUTPUT); // charge relay
  pinMode(OUT4, OUTPUT); // Negative contactor
  pinMode(OUT5, OUTPUT); // pwm driver output
  pinMode(OUT6, OUTPUT); // pwm driver output
  pinMode(OUT7, OUTPUT); // pwm driver output
  pinMode(OUT8, OUTPUT); // pwm driver output
  pinMode(led, OUTPUT);

  analogWriteFrequency(OUT5, pwmfreq);
  analogWriteFrequency(OUT6, pwmfreq);
  analogWriteFrequency(OUT7, pwmfreq);
  analogWriteFrequency(OUT8, pwmfreq);

  EEPROM.get(0, settings);
  // ONE-TIME RESET (EEPROM 0x1A): with any other version all settings go to the
  // defaults from loadSettings(). Only the CAN speed is kept, otherwise the
  // connection to the GX would drop (e.g. 250 kbit/s on VE.Can for Lynx current).
  if (settings.version == EEPROM_VERSION_VIC5)
  {
    // older version: keep all settings, only add the new field
    settings.JsonOut = 0;
    settings.version = EEPROM_VERSION_VIC6;
  }
  if (settings.version == EEPROM_VERSION_VIC6)
  {
    // v26.24: normal range 3.00 - 4.10 V. Only when the OLD defaults are still set:
    // DCL = 0 at 3.00 V (was 3.20 V), low voltage warning at 2.90 V (was 3.00 V + WarnOff).
    if (fabsf(settings.DischVsetpoint - 3.2f) < 0.001f && fabsf(settings.UnderVSetpoint - 3.0f) < 0.001f)
    {
      settings.DischVsetpoint = 3.0f;
      settings.UnderVSetpoint = 2.9f;
    }
    settings.version = EEPROM_VERSION_VIC7;
  }
  if (settings.version == EEPROM_VERSION_VIC7)
  {
    // v26.28: storage mode via input IN1 only when switched on explicitly (default off)
    settings.StorageInput = 0;
    settings.version = EEPROM_VERSION_VIC8;
  }
  if (settings.version == EEPROM_VERSION_VIC8)
  {
    // v26.29: CAN protocol to the inverter, default Victron
    settings.InverterProto = 0;
    settings.version = EEPROM_VERSION_VIC9;
  }
  if (settings.version == EEPROM_VERSION_VIC9)
  {
    // v26.34: cell readings below 1.0 V cannot be real (sensor fault); only when still the old default
    if (fabsf(settings.IgnoreVolt - 0.5f) < 0.001f) settings.IgnoreVolt = 1.0f;
    settings.version = EEPROM_VERSION;
    EEPROM.put(0, settings);
  }
  else if (settings.version != EEPROM_VERSION)
  {
    uint32_t oldCanSpeed = settings.canSpeed;
    bool knownOld = (settings.version >= EEPROM_VERSION_V2 && settings.version < EEPROM_VERSION);
    loadSettings();
    if (knownOld && (oldCanSpeed == 125000 || oldCanSpeed == 250000 || oldCanSpeed == 500000 || oldCanSpeed == 1000000))
    {
      settings.canSpeed = oldCanSpeed;
    }
    EEPROM.put(0, settings);
    eepromWasReset = true;
  }
  if (settings.ScaleLimits > 1) settings.ScaleLimits = 1;
  if (settings.AutoTopology > 1) settings.AutoTopology = 1;
  if (settings.CVLControl > 1) settings.CVLControl = 1;
  if (settings.StorageInput > 1) settings.StorageInput = 0;
  if (settings.InverterProto > 1) settings.InverterProto = 0;
  if (settings.JsonOut > 1) settings.JsonOut = 0;
  if (settings.JsonOut == 1) debug = 0; // JSON mode: no text output in between
  if (!(settings.balanceStart >= 0.002f && settings.balanceStart <= 0.5f)) settings.balanceStart = 0.015f;

  bmsCan.begin(settings.canSpeed);   // all standard and extended identifiers (hal.cpp)



  //if using enable pins on a transceiver they need to be set on


  adc->adc0->setAveraging(16); // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc0->startContinuous(ACUR1);


  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.println("Starting up!");
  SERIALCONSOLE.print("TeslaBMS_Victron (Claude Tesla BMS) on "); SERIALCONSOLE.println(BOARD_NAME);
  SERIALCONSOLE.print("Firmware v");
  SERIALCONSOLE.print(FW_YEAR);
  SERIALCONSOLE.print(".");
  SERIALCONSOLE.print(FW_BUILD & 0xFF);
  SERIALCONSOLE.print("  Compiled: ");
  SERIALCONSOLE.print(__DATE__);
  SERIALCONSOLE.print(" ");
  SERIALCONSOLE.println(__TIME__);
  calcFirmwareDate();

  SERIALDISP.begin(115200); //display and can adpater canbus

  // Display reason the Teensy was last reset
  Serial.println();
  Serial.println("Reason for last Reset: ");

  halPrintResetReason(Serial);
  Serial.println();
  ///////////////////


  // enable WDT
  halWatchdogStart();   // about 5.5 s (hal.cpp)
  /////////////////


  //VE.begin(19200); //Victron VE direct bus
#if defined (__arm__) && defined (__SAM3X8E__)
  serialSpecialInit(USART0, 612500); //required for Due based boards as the stock core files don't support 612500 baud.
#endif

  SERIALCONSOLE.println("Started serial interface to BMS.");

  /*
    EEPROM.get(0, settings);
    if (settings.version != EEPROM_VERSION)
    {
      loadSettings();
    }
  */

  Logger::setLoglevel(Logger::Off); //Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  lastUpdate = 0;
  // NEW: detection. Auto: count at every start. Otherwise: fixed number (Pstrings).
  bms.setTopology(settings.Scells, settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
  if (settings.AutoTopology == 1)
  {
    autoDetectTopology();
  }
  else
  {
    bms.startupDetect();
  }
  digitalWrite(led, HIGH);

  //SOC recovery//

  SOC = (EEPROM.read(1000));
  if (settings.voltsoc == 1)
  {
    SOCmem = 0;
  }
  else
  {
    if (SOC > 100)
    {
      SOCmem = 0;
    }
    else if (SOC > 1)
    {
      //SOCmem = 1;
    }
  }


  SERIALCONSOLE.println("Recovery SOC: ");
  SERIALCONSOLE.print(SOC);

  ////Calculate fixed numbers
  pwmcurmin = (pwmcurmid / 50 * pwmcurmax * -1);
  ////
  bms.clearFaults();

  printKeySettings();

  ///precharge timer kickers
  Pretimer = millis();
  Pretimer1  = millis();

  // setup interrupts
  //RISING/HIGH/CHANGE/LOW/FALLING
  attachInterrupt (IN4, isrCP , CHANGE); // attach BUTTON 1 interrupt handler [ pin# 7 ]

  halLowVoltageStart(low_voltage_isr);   // Teensy 3.2: save SOC at low supply voltage (hal.cpp)

    bmsstatus = Boot;
}

void loop()
{

  bmsCan.poll();   // CAN error registers (bus health, JSON "cb")
  while (bmsCan.read(inMsg))
  {
    canread();
  }

  if (SERIALCONSOLE.available() > 0)
  {
    lastMenuInput = millis();
    menu();
  }
  // NEW: close the menu automatically after 2 minutes without input
  // (e.g. when another device on the USB port has sent stray bytes)
  if (menuload != 0 && (millis() - lastMenuInput) > MENU_TIMEOUT)
  {
    menuload = 0;
    debug = (settings.JsonOut == 1) ? 0 : 1;
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("Menu closed (2 minutes without input, changes not saved)");
  }
  if (outputcheck != 1)
  {
    contcon();
    if (settings.ESSmode == 1)
    {
      if (settings.ChargerDirect == 1)
      {
        OutputEnable = 1;
      }
      else
      {
        if (digitalRead(IN2) == HIGH || CanOnReq == true)
        {
          OutputEnable = 1;
          //Serial.println(CanOnReq);
        }
        else
        {
          OutputEnable = 0;
        }
      }
      if (bmsstatus != Error && bmsstatus != Boot && OutputEnable == 1)
      {
        contctrl = contctrl | 4; //turn on negative contactor
        if (settings.tripcont != 0)
        {
          if (true)   // ESS: always, also at over/undervoltage - CCL/DCL protect the battery, it must stay chargeable and dischargeable
          {
            if (halOutRead(OUT2) == LOW && halOutRead(OUT4) == LOW)
            {
              mainconttimer = millis();
              halOutWrite(OUT4, HIGH);//Precharge start
              Serial.println();
              Serial.println("Precharge!!!");
              Serial.println(mainconttimer);
              Serial.println();
            }
            if (mainconttimer + settings.Pretime < millis() && halOutRead(OUT2) == LOW && abs(currentact) < settings.Precurrent)
            {
              halOutWrite(OUT2, HIGH);//turn on contactor
              contctrl = contctrl | 2; //turn on contactor
              Serial.println();
              Serial.println("Main On!!!");
              Serial.println();
              mainconttimer = millis() + settings.Pretime;
            }
            if (mainconttimer + settings.Pretime + 1000 < millis() )
            {
              halOutWrite(OUT4, LOW);//ensure precharge is low
            }
          }
          else
          {
            halOutWrite(OUT4, LOW);//ensure precharge is low
            mainconttimer = 0;
          }
        }
        // Storage mode (CVL = StoreVsetpoint x Scells, e.g. 3.80 V per cell) only when switched on
        // in the menu (m -> o) AND input IN1 is high. Default off: a floating/disturbed IN1 on a
        // board without wiring may never lower the CVL of a full battery.
        if (settings.StorageInput != 1 || digitalRead(IN1) == LOW)//Key OFF
        {
          if (storagemode == 1)
          {
            storagemode = 0;
          }
        }
        else
        {
          if (storagemode == 0)
          {
            storagemode = 1;
          }
        }
        if (bms.getHighCellVolt() > settings.balanceVoltage) // NEW: the difference per cell is assessed per string in balanceCells()
        {
          balancecells = 1;
        }
        else
        {
          balancecells = 0;
        }

        //Pretimer + settings.Pretime > millis();

        if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > settings.StoreVsetpoint || chargecurrent == 0)
          {
            halOutWrite(OUT3, LOW);//turn off charger
            // contctrl = contctrl & 253;
            // Pretimer = millis();
            Charged = 1;
            SOCcharged(2);
          }
          else
          {
            if (Charged == 1)
            {
              if (bms.getHighCellVolt() < (settings.StoreVsetpoint - settings.ChargeHys))
              {
                Charged = 0;
                halOutWrite(OUT3, HIGH);//turn on charger
                /*
                  if (Pretimer + settings.Pretime < millis())
                  {
                  contctrl = contctrl | 2;
                  Pretimer = 0;
                  }
                */
              }
            }
            else
            {
              halOutWrite(OUT3, HIGH);//turn on charger
              /*
                if (Pretimer + settings.Pretime < millis())
                {
                contctrl = contctrl | 2;
                Pretimer = 0;
                }
              */
            }
          }
        }
        else
        {
          if (bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighCellVolt() > settings.ChargeVsetpoint || chargecurrent == 0)
          {
            if ((millis() - overtriptimer) > settings.triptime)
            {
              if (halOutRead(OUT3) == 1)
              {
                Serial.println();
                Serial.println("Over Voltage Trip");
                halOutWrite(OUT3, LOW);//turn off charger
                // contctrl = contctrl & 253;
                //Pretimer = millis();
                Charged = 1;
                SOCcharged(2);
              }

            }
          }
          else
          {
            overtriptimer = millis();
            if (Charged == 1)
            {

              if (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys))
              {
                if (halOutRead(OUT3) == 0)
                {
                  Serial.println();
                  Serial.println("Reset Over Voltage Trip Not Charged");
                  Charged = 0;
                  halOutWrite(OUT3, HIGH);//turn on charger
                }
                /*
                  if (Pretimer + settings.Pretime < millis())
                  {
                  // Serial.println();
                  //Serial.print(Pretimer);
                  contctrl = contctrl | 2;
                  }*/
              }

            }
            else
            {
              if (halOutRead(OUT3) == 0)
              {
                Serial.println();
                Serial.println("Reset Over Voltage Trip Not Charged");
                halOutWrite(OUT3, HIGH);//turn on charger
              }
              /*
                if (Pretimer + settings.Pretime < millis())
                {
                // Serial.println();
                //Serial.print(Pretimer);
                contctrl = contctrl | 2;
                }*/
            }
          }
        }

        if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
        {
          if (halOutRead(OUT1) == 1)
          {

            if ((millis() - undertriptimer) > settings.triptime)
            {
              Serial.println();
              Serial.println("Under Voltage Trip");
              halOutWrite(OUT1, LOW);//turn off discharge
              // contctrl = contctrl & 254;
              // Pretimer1 = millis();
            }
          }
        }
        else
        {
          undertriptimer = millis();

          if (bms.getLowCellVolt() > settings.DischVsetpoint + settings.DischHys)
          {
            if (halOutRead(OUT1) == 0)
            {
              Serial.println();
              Serial.println("Reset Under Voltage Trip");
              halOutWrite(OUT1, HIGH);//turn on discharge
            }
            /*
              if (Pretimer1 + settings.Pretime < millis())
              {
              contctrl = contctrl | 1;
              }*/
          }
        }

        if (SOCset == 1)
        {
          if (settings.tripcont == 0)
          {
            // Cell voltage is NEVER an Error in ESS mode (Error sets CCL and DCL to 0):
            // too high -> CCL = 0, too low -> DCL = 0, see currentlimit(). Only temperature.
            if (bms.getHighTemperature() > settings.OverTSetpoint)
            {
              halOutWrite(OUT2, HIGH);//trip breaker
              bmsstatus = Error;
            }
            else
            {
              halOutWrite(OUT2, LOW);//trip breaker
            }
          }
          else
          {
            if (bms.getHighTemperature() > settings.OverTSetpoint)  // not for cell voltage, see above
            {
              halOutWrite(OUT2, LOW);//turn off contactor
              contctrl = contctrl & 253; //turn off contactor
              halOutWrite(OUT4, LOW);//ensure precharge is low
              bmsstatus = Error;
            }
          }
        }
      }
      else
      {
        //halOutWrite(OUT2, HIGH);//trip breaker
        Discharge = 0;
        halOutWrite(OUT4, LOW);
        halOutWrite(OUT3, LOW);//turn off charger
        halOutWrite(OUT2, LOW);
        halOutWrite(OUT1, LOW);//turn off discharge
        contctrl = 0; //turn off out 5 and 6

        if (SOCset == 1)
        {
          if (settings.tripcont == 0)
          {

            halOutWrite(OUT2, HIGH);//trip breaker
          }
          else
          {
            halOutWrite(OUT2, LOW);//turn off contactor
            halOutWrite(OUT4, LOW);//ensure precharge is low
          }

          if (bms.getHighTemperature() < settings.OverTSetpoint && bms.getStringsOnline() > 0) // NEW: at least 1 complete string; cell voltage is no Error in ESS mode
          {
            bmsstatus = Ready;
          }
        }
      }
      //pwmcomms();
    }
    else
    {
      switch (bmsstatus)
      {
        case (Boot):
          Discharge = 0;
          halOutWrite(OUT4, LOW);
          halOutWrite(OUT3, LOW);//turn off charger
          halOutWrite(OUT2, LOW);
          halOutWrite(OUT1, LOW);//turn off discharge
          contctrl = 0;
          bmsstatus = Ready;
          break;

        case (Ready):
          Discharge = 0;
          halOutWrite(OUT4, LOW);
          halOutWrite(OUT3, LOW);//turn off charger
          halOutWrite(OUT2, LOW);
          halOutWrite(OUT1, LOW);//turn off discharge
          contctrl = 0; //turn off out 5 and 6
          accurlim = 0;
          if (bms.getHighCellVolt() > settings.balanceVoltage) // NEW: the difference per cell is assessed per string in balanceCells()
          {
            //bms.balanceCells();
            balancecells = 1;
          }
          else
          {
            balancecells = 0;
          }
          if (digitalRead(IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys)) && bms.getHighTemperature() < (settings.OverTSetpoint - settings.WarnToff)) //detect AC present for charging and check not balancing
          {
            if (settings.ChargerDirect == 1)
            {
              bmsstatus = Charge;
            }
            else
            {
              bmsstatus = Precharge;
              Pretimer = millis();
            }
          }
          if (digitalRead(IN1) == HIGH && bms.getLowCellVolt() > settings.DischVsetpoint) //detect Key ON
          {
            bmsstatus = Precharge;
            Pretimer = millis();
          }

          break;

        case (Precharge):
          Discharge = 0;
          Prechargecon();
          break;


        case (Drive):
          Discharge = 1;
          accurlim = 0;
          if (digitalRead(IN1) == LOW)//Key OFF
          {
            bmsstatus = Ready;
          }
          if (digitalRead(IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys)) && bms.getHighTemperature() < (settings.OverTSetpoint - settings.WarnToff)) //detect AC present for charging and check not balancing
          {
            bmsstatus = Charge;
          }

          break;

        case (Charge):
          if (settings.ChargerDirect > 0)
          {
            Discharge = 0;
            halOutWrite(OUT4, LOW);
            halOutWrite(OUT2, LOW);
            halOutWrite(OUT1, LOW);//turn off discharge
            contctrl = 0; //turn off out 5 and 6
          }
          Discharge = 0;
          if (digitalRead(IN2) == HIGH)
          {
            chargecurrentlimit = true;
          }
          else
          {
            chargecurrentlimit = false;
          }
          halOutWrite(OUT3, HIGH);//enable charger
          if (bms.getHighCellVolt() > settings.balanceVoltage)
          {
            //bms.balanceCells();
            balancecells = 1;
          }
          else
          {
            balancecells = 0;
          }
          if (bms.getHighCellVolt() > settings.ChargeVsetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
          {
            if (bms.getAvgCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
            {
              SOCcharged(2);
            }
            else
            {
              SOCcharged(1);
            }
            halOutWrite(OUT3, LOW);//turn off charger
            bmsstatus = Ready;
          }
          if (digitalRead(IN3) == LOW)//detect AC not present for charging
          {
            bmsstatus = Ready;
          }
          break;

        case (Error):
          Discharge = 0;
          halOutWrite(OUT4, LOW);
          halOutWrite(OUT3, LOW);//turn off charger
          halOutWrite(OUT2, LOW);
          halOutWrite(OUT1, LOW);//turn off discharge
          contctrl = 0; //turn off out 5 and 6
          /*
                    if (digitalRead(IN3) == HIGH) //detect AC present for charging
                    {
                      bmsstatus = Charge;
                    }
          */
            if (bms.getLowCellVolt() >= settings.UnderVSetpoint && bms.getHighCellVolt() <= settings.OverVSetpoint && digitalRead(IN1) == LOW)
            {
              bmsstatus = Ready;
            }
            
          break;
      }
    }
    if ( settings.cursens == Analoguedual || settings.cursens == Analoguesing)
    {
      getcurrent();
    }
  }

  if (millis() - looptime > 500)
  {
    looptime = millis();
    unsigned long blockStart = millis();
    bms.getAllVoltTemp();
    //UV  check
    if (settings.ESSmode == 1)
    {
      // Cell voltage gives NO Error in ESS mode (Error would set CCL and DCL to 0):
      // - too high: CCL = 0 (overVoltLatch in currentlimit()), discharging stays allowed
      // - too low:  DCL = 0, charging stays allowed (slowly below UnderVSetpoint)
    }
    else //In 'vehicle' mode
    {
      if (bms.getLowCellVolt() < settings.UnderVSetpoint)
      {
        if (UnderTimer < millis()) //check is last time not undervoltage is longer thatn UnderDur ago
        {
          bmsstatus = Error;
        }
      }
      else
      {
        UnderTimer = millis() + settings.triptime;
      }

      if (bms.getHighCellVolt() < settings.UnderVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
      {
        bmsstatus = Error;
      }

      if (bms.getHighCellVolt() > settings.OverVSetpoint)
      {
        if (OverTime < millis()) //check is last time not undervoltage is longer thatn UnderDur ago
        {
          bmsstatus = Error;
        }
      }
      else
      {
        OverTime = millis() + settings.triptime;
      }
    }

    balancing();

    if (debug != 0)
    {
      printbmsstat();
      bms.printPackDetails(debugdigits);
    }
    if (CSVdebug != 0)
    {
      bms.printAllCSV(millis(), currentact, SOC, delim);
    }
    if (inputcheck != 0)
    {
      inputdebug();
    }

    if (outputcheck != 0)
    {
      outputdebug();
    }
    else
    {
      gaugeupdate();
    }
    updateSOC();
    currentlimit();
    cvlControl();

    if (settings.ESSmode == 1 && settings.ChargerDirect == 0 && CanOnRev == true)
    {
      if ((millis() - CanOntimeout) > 5000)
      {
        Serial.println();
        Serial.println("0x309 Can On Request Missing");
        CanOnReq = false;
      }
    }


    // NEW: module/string monitoring instead of "Series Cells Fault"
    cellspresent = bms.seriescells();
    modulecheck();
    alarmupdate();
    if (settings.JsonOut == 1 && menuload == 0 && (millis() - jsonTimer) >= JSON_INTERVAL)
    {
      jsonTimer = millis();
      printJson(); // 1x per 5 seconds (dashboard; keep the load on the GX low)
    }
    if (CSVdebug != 1)
    {
      dashupdate();
    }
    lastBlockMs = millis() - blockStart; // duration of measuring + balancing + processing
    resetwdog();
  }

  if (millis() - cleartime > 5000)
  {
    bms.clearmodules();
    cleartime = millis();
  }

  if (millis() - looptime1 > settings.chargerspd)
  {
    looptime1 = millis();
    if (settings.InverterProto == 1) lgResuCan();   // SolarEdge StorEdge RWS (LG RESU LV emulation)
    else VEcan();                                  // Victron
    if (settings.ESSmode == 1)
    {
      chargercomms();
    }
    else
    {
      if (bmsstatus == Charge)
      {
        chargercomms();
      }
    }
  }
}

/*
  Alarms and warnings for 0x35A. Per alarm 2 bits:
    1 (binary 01) = active, 2 (binary 10) = OK, 0 (binary 00) = not supported.
  Supported alarms ALWAYS send 1 or 2; unsupported ones stay 0, so VenusOS
  can hide them. (Previously 0 was sent when OK, which reads as "not supported".)
*/
static inline uint8_t almPair(bool active, uint8_t shift)
{
  return (uint8_t)((active ? 1 : 2) << shift);
}

void alarmupdate()
{
  bool noData = (bms.getStringsOnline() == 0);
  float hc = bms.getHighCellVolt();
  float lc = bms.getLowCellVolt();
  float ht = bms.getHighTemperature();
  float lt = bms.getLowTemperature();

  ///// alarms /////
  // byte 0: bits 0-1 general (not supported), 2-3 high voltage, 4-5 low voltage, 6-7 high temperature
  // High voltage: a full battery (ChargeVsetpoint, e.g. 4.10 V) is normal operation and gives
  // no warning. Warning only at real overvoltage (OverVSetpoint, 4.20 V, where CCL = 0),
  // alarm at OverVSetpoint + OV_ALARM_MARGIN (4.25 V).
  // Low voltage (mirrored): normal down to DischVsetpoint (3.00 V, DCL = 0), warning below
  // UnderVSetpoint (2.90 V), alarm below UnderVSetpoint - OV_ALARM_MARGIN (2.85 V).
  alarm[0] = almPair(hc >= settings.OverVSetpoint + OV_ALARM_MARGIN, 2)
           | almPair(lc < settings.UnderVSetpoint - OV_ALARM_MARGIN, 4)
           | almPair(ht > settings.OverTSetpoint, 6);
  // byte 1: bits 0-1 low temperature; 2-3 high temp charge, 4-5 low temp charge, 6-7 high current: not supported
  alarm[1] = almPair(lt < settings.UnderTSetpoint, 0);
  // byte 2: bits 6-7 BMS internal (missing module/string); high charge current, contactor, short circuit: not supported
  alarm[2] = almPair(moduleAlarm || noData, 6);
  // byte 3: bits 0-1 cell imbalance
  alarm[3] = almPair((hc - lc) > settings.CellGap, 0);

  ///// warnings (same layout) /////
  warning[0] = almPair(hc >= settings.OverVSetpoint, 2)
             | almPair(lc < settings.UnderVSetpoint, 4)
             | almPair(ht > (settings.OverTSetpoint - settings.WarnToff), 6);
  warning[1] = almPair(lt < (settings.UnderTSetpoint + settings.WarnToff), 0);
  warning[2] = almPair(moduleWarning || noData, 6);
  warning[3] = 0; // cell imbalance warning: not supported (only the alarm)
}

/*
  NEW: monitoring of modules and strings.
  - fewer strings online than set -> moduleWarning immediately
  - longer than settings.ModuleAlarmDelay -> moduleAlarm
  - every RESCAN_INTERVAL ms search for the missing modules again
  No Error status: CVL/CCL/DCL keep going to the GX (CCL/DCL scaled in currentlimit()).
*/
void modulecheck()
{
  bool incomplete = (bms.getStringsOnline() < bms.getStringsExpected());

  if (incomplete)
  {
    if (!moduleWarning)
    {
      moduleWarning = true;
      moduleMissingSince = millis();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("!!! Module/string missing !!!");
      bms.printTopology();
    }
    if (!moduleAlarm && (millis() - moduleMissingSince) > (unsigned long)settings.ModuleAlarmDelay * 1000UL)
    {
      moduleAlarm = true;
      SERIALCONSOLE.println("!!! Module alarm active (delay elapsed) !!!");
    }
  }
  else
  {
    if (moduleWarning)
    {
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("All strings complete again");
    }
    moduleWarning = false;
    moduleAlarm = false;
  }

  if ((incomplete || bms.getModulesOnline() < bms.getExpectedModules()) && (millis() - rescanTimer) > RESCAN_INTERVAL)
  {
    rescanTimer = millis();
    bms.rescan();
  }
}

void gaugeupdate()
{
  if (gaugedebug == 1)
  {
    SOCtest = SOCtest + 10;
    if (SOCtest > 1000)
    {
      SOCtest = 0;
    }
    analogWrite(OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
    if (debug != 0)
    {
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("SOC : ");
      SERIALCONSOLE.print(SOCtest * 0.1);
      SERIALCONSOLE.print("  fuel pwm : ");
      SERIALCONSOLE.print(map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
      SERIALCONSOLE.println("  ");
    }
  }
  if (gaugedebug == 2)
  {
    SOCtest = 0;
    analogWrite(OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
  if (gaugedebug == 3)
  {
    SOCtest = 1000;
    analogWrite(OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
  if (gaugedebug == 0)
  {
    analogWrite(OUT8, map(SOC, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
}

void printbmsstat()
{
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Strings online: ");
  SERIALCONSOLE.print(bms.getStringsOnline());
  SERIALCONSOLE.print("/");
  SERIALCONSOLE.print(bms.getStringsExpected());
  SERIALCONSOLE.print("  Modules: ");
  SERIALCONSOLE.print(bms.getModulesOnline());
  SERIALCONSOLE.print("/");
  SERIALCONSOLE.print(bms.getExpectedModules());
  SERIALCONSOLE.print("  CVL: ");
  SERIALCONSOLE.print(cvlActual, 1);
  SERIALCONSOLE.print("V  CCL: ");
  SERIALCONSOLE.print(chargecurrent * 0.1, 1);
  SERIALCONSOLE.print("A  DCL: ");
  SERIALCONSOLE.print(discurrent * 0.1, 1);
  SERIALCONSOLE.print("A");
  if (overVoltLatch) SERIALCONSOLE.print("  OVERVOLTAGE: CCL=0");
  if (moduleAlarm) SERIALCONSOLE.print("  MODULE ALARM");
  else if (moduleWarning) SERIALCONSOLE.print("  MODULE WARNING");
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMS Status : ");
  if (settings.ESSmode == 1)
  {
    SERIALCONSOLE.print("ESS Mode ");

    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      SERIALCONSOLE.print(": UnderVoltage ");
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      SERIALCONSOLE.print(": OverVoltage ");
    }
    if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
    {
      SERIALCONSOLE.print(": Cell Imbalance ");
    }
    if (bms.getAvgTemperature() > settings.OverTSetpoint)
    {
      SERIALCONSOLE.print(": Over Temp ");
    }
    if (bms.getAvgTemperature() < settings.UnderTSetpoint)
    {
      SERIALCONSOLE.print(": Under Temp ");
    }
    if (storagemode == 1)
    {
      if (bms.getLowCellVolt() > settings.StoreVsetpoint)
      {
        SERIALCONSOLE.print(": OverVoltage Storage ");
        SERIALCONSOLE.print(": UNhappy:");
      }
      else
      {
        SERIALCONSOLE.print(": Happy ");
      }
    }
    else
    {
      if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
      {

        if ( bmsstatus == Error)
        {
          SERIALCONSOLE.print(": UNhappy:");
        }
        else
        {
          SERIALCONSOLE.print(": Happy ");
        }
      }
    }
  }
  else
  {
    SERIALCONSOLE.print(bmsstatus);
    switch (bmsstatus)
    {
      case (Boot):
        SERIALCONSOLE.print(" Boot ");
        break;

      case (Ready):
        SERIALCONSOLE.print(" Ready ");
        break;

      case (Precharge):
        SERIALCONSOLE.print(" Precharge ");
        break;

      case (Drive):
        SERIALCONSOLE.print(" Drive ");
        break;

      case (Charge):
        SERIALCONSOLE.print(" Charge ");
        break;

      case (Error):
        SERIALCONSOLE.print(" Error ");
        break;
    }
  }
  SERIALCONSOLE.print("  ");
  if (digitalRead(IN3) == HIGH)
  {
    SERIALCONSOLE.print("| AC Present |");
  }
  if (digitalRead(IN1) == HIGH)
  {
    SERIALCONSOLE.print("| Key ON |");
  }
  if (balancecells == 1)
  {
    SERIALCONSOLE.print("|Balancing Active: ");
    SERIALCONSOLE.print(bms.getBalancing());
    SERIALCONSOLE.print(" cells");
  }
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(cellspresent);
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Out:");
  SERIALCONSOLE.print(halOutRead(OUT1));
  SERIALCONSOLE.print(halOutRead(OUT2));
  SERIALCONSOLE.print(halOutRead(OUT3));
  SERIALCONSOLE.print(halOutRead(OUT4));
  SERIALCONSOLE.print(" Cont:");
  if ((contstat & 1) == 1)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 2) == 2)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 4) == 4)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 8) == 8)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  SERIALCONSOLE.print(" In:");
  SERIALCONSOLE.print(digitalRead(IN1));
  SERIALCONSOLE.print(digitalRead(IN2));
  SERIALCONSOLE.print(digitalRead(IN3));
  SERIALCONSOLE.print(digitalRead(IN4));


  SERIALCONSOLE.print(" Charge Current Limit : ");
  SERIALCONSOLE.print(chargecurrent * 0.1, 0);
  SERIALCONSOLE.print(" A DisCharge Current Limit : ");
  SERIALCONSOLE.print(discurrent * 0.1, 0);
  SERIALCONSOLE.print(" A");

  if (bmsstatus == Charge || accurlim > 0)
  {
    Serial.print("  CP AC Current Limit: ");
    Serial.print(accurlim);
    Serial.print(" A");
  }

  if (bmsstatus == Charge && CPdebug == 1)
  {
    Serial.print("A  CP Dur: ");
    Serial.print(duration);
    Serial.print("  Charge Power : ");
    Serial.print(chargerpower);
    if (chargecurrentlimit == false)
    {
      SERIALCONSOLE.print("  No Charge Current Limit");
    }
    else
    {
      SERIALCONSOLE.print("  Charge Current Limit Active");
    }
  }
}


void getcurrent()
{
  if ( settings.cursens == Analoguedual || settings.cursens == Analoguesing)
  {
    if ( settings.cursens == Analoguedual)
    {
      if (currentact < settings.changecur && currentact > (settings.changecur * -1))
      {
        sensor = 1;
        adc->adc0->startContinuous(ACUR1);
      }
      else
      {
        sensor = 2;
        adc->adc0->startContinuous(ACUR2);
      }
    }
    else
    {
      sensor = 1;
      adc->adc0->startContinuous(ACUR1);
    }
    if (sensor == 1)
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.println();
        if ( settings.cursens == Analoguedual)
        {
          SERIALCONSOLE.print("Low Range: ");
        }
        else
        {
          SERIALCONSOLE.print("Single In: ");
        }
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3300 / adc->adc0->getMaxValue()); //- settings.offset1)
        SERIALCONSOLE.print(" ");
        SERIALCONSOLE.print(settings.offset1);
      }
      RawCur = int16_t((value * 3300 / adc->adc0->getMaxValue()) - settings.offset1) / (settings.convlow * 0.0000066);

      if (abs((int16_t(value * 3300 / adc->adc0->getMaxValue()) - settings.offset1)) <  settings.CurDead)
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(int16_t(value * 3300 / adc->adc0->getMaxValue()) - settings.offset1);
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print(" mA");
        SERIALCONSOLE.print("  ");
      }
    }
    else
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("High Range: ");
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3300 / adc->adc0->getMaxValue() );//- settings.offset2)
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(settings.offset2);
      }
      RawCur = int16_t((value * 3300 / adc->adc0->getMaxValue()) - settings.offset2) / (settings.convhigh *  0.0000066);
      if (value < 100 || value > (adc->adc0->getMaxValue() - 100))
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print((float(value * 3300 / adc->adc0->getMaxValue()) - settings.offset2));
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print("mA");
        SERIALCONSOLE.print("  ");
      }
    }
  }

  if (settings.invertcur == 1)
  {
    RawCur = RawCur * -1;
  }

  lowpassFilter.input(RawCur);
  if (debugCur != 0)
  {
    SERIALCONSOLE.print(lowpassFilter.output());
    SERIALCONSOLE.print(" | ");
    SERIALCONSOLE.print(settings.changecur);
    SERIALCONSOLE.print(" | ");
  }

  currentact = lowpassFilter.output();

  if (debugCur != 0)
  {
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA  ");
  }

  if ( settings.cursens == Analoguedual)
  {
    if (sensor == 1)
    {
      if (currentact > 500 || currentact < -500 )
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
    if (sensor == 2)
    {
      if (currentact > settings.changecur || currentact < (settings.changecur * -1) )
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
  }
  else
  {
    if (currentact > 500 || currentact < -500 )
    {
      ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
      lasttime = millis();
    }
    else
    {
      lasttime = millis();
    }
  }
  currentact = settings.ncur * currentact;
  RawCur = 0;
  /*
    AverageCurrentTotal = AverageCurrentTotal - RunningAverageBuffer[NextRunningAverage];

    RunningAverageBuffer[NextRunningAverage] = currentact;

    if (debugCur != 0)
    {
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(RunningAverageBuffer[NextRunningAverage]);
      SERIALCONSOLE.print(" | ");
    }
    AverageCurrentTotal = AverageCurrentTotal + RunningAverageBuffer[NextRunningAverage];
    if (debugCur != 0)
    {
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
    }

    NextRunningAverage = NextRunningAverage + 1;

    if (NextRunningAverage > RunningAverageCount)
    {
      NextRunningAverage = 0;
    }

    AverageCurrent = AverageCurrentTotal / (RunningAverageCount + 1);

    if (debugCur != 0)
    {
      SERIALCONSOLE.print(AverageCurrent);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(NextRunningAverage);
    }
  */
}


/*
  NEW: SOC from the cell voltage when there is no current measurement (e.g. no Lynx shunt).
  Open circuit voltage curve of Tesla NCA cells, scaled to the working range
  3.00 V (0 %) .. 4.10 V (100 %), on the AVERAGE cell voltage, low-pass filtered
  (time constant 120 s) so that short load dips do not make the SOC jump.
*/
static const float ocvV[]   = {3.00f, 3.30f, 3.45f, 3.55f, 3.62f, 3.68f, 3.75f, 3.83f, 3.91f, 3.98f, 4.06f, 4.10f};
static const float ocvSoc[] = {0.0f,  4.0f,  9.0f, 19.0f, 29.0f, 40.0f, 51.0f, 62.0f, 73.0f, 83.0f, 94.0f, 100.0f};
float socVoltFiltered = -1.0f;
uint32_t socVoltMs = 0;

float socFromOcv(float v)
{
  const int n = sizeof(ocvV) / sizeof(ocvV[0]);
  if (v <= ocvV[0]) return 0.0f;
  if (v >= ocvV[n - 1]) return 100.0f;
  for (int i = 1; i < n; i++)
  {
    if (v < ocvV[i]) return ocvSoc[i - 1] + (ocvSoc[i] - ocvSoc[i - 1]) * (v - ocvV[i - 1]) / (ocvV[i] - ocvV[i - 1]);
  }
  return 100.0f;
}

void updateSOC() {
   if (SOCreset == 1) {
    SOC = map(uint16_t(bms.getLowCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);
    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
    SOCreset = 0;
  }

  if (SOCset == 0 && SOCmem == 0) {
    if (millis() > 5000) {
      SOC = map(uint16_t(bms.getLowCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

      ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
      SOCset = 1;
      if (debug != 0) {
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("//////////////////////////////////////// SOC SET ////////////////////////////////////////");
      }
      if (settings.ESSmode == 1) {
        bmsstatus = Ready;
      }
    }
  }

  if (SOCset == 0 && SOCmem == 1) {
    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
    if (millis() > 5000) {
      SOCset = 1;
      if (debug != 0) {
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("//////////////////////////////////////// SOC SET ////////////////////////////////////////");
      }
      if (settings.ESSmode == 1) {
        bmsstatus = Ready;
      }
    }
  }

  SOC = ((ampsecond * 0.27777777777778) / (settings.CAP * settings.Pstrings * 1000)) * 100;

  // NEW: current via CAN but none received for 30 s (e.g. on a SolarEdge without Lynx/shunt):
  // the coulomb count would freeze, so take the SOC from the cell voltage instead.
  socFromVoltage = (settings.voltsoc == 1 || settings.cursens == 0 ||
                    (settings.cursens == Canbus && (lastCurrentMs == 0 || millis() - lastCurrentMs > 30000)));
  if (socFromVoltage && settings.voltsoc != 1 && bms.getStringsOnline() > 0) {
    // no current measurement: OCV curve on the average cell voltage, filtered
    float target = socFromOcv(bms.getAvgCellVolt());
    uint32_t now = millis();
    if (socVoltFiltered < 0.0f) socVoltFiltered = target;
    else
    {
      float dt = (now - socVoltMs) / 1000.0f;
      if (dt > 10.0f) dt = 10.0f;
      socVoltFiltered += (target - socVoltFiltered) * (dt / 120.0f);
    }
    socVoltMs = now;
    SOC = (int)(socVoltFiltered + 0.5f);
    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
  }
  else if (socFromVoltage) {
    // explicitly set to voltage SOC (menu): the configured socvolt line on the lowest cell, as before
    SOC = map(uint16_t(bms.getLowCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
  }

  // NEW: SOC from outside (Lynx/shunt on the GX, sent by Node-RED over USB) has priority while it is
  // fresh (< 60 s); otherwise the own calculation above (coulomb count or cell voltage) is used.
  socExternal = (extSoc >= 0.0f && extSocMs != 0 && millis() - extSocMs < 60000);
  if (socExternal) {
    SOC = (int)(extSoc + 0.5f);
    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
    socFromVoltage = false;
  }

  if (!socFromVoltage) { socVoltFiltered = (float)SOC; socVoltMs = millis(); }   // smooth hand-over if the current measurement drops out

  if (SOC >= 100) {
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778;  //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
    SOC = 100;
  }


  if (SOC < 0) {
    SOC = 0;  //reset SOC this way the can messages remain in range for other devices. Ampseconds will keep counting.
  }

  if (debug != 0) {
    if (settings.cursens == Analoguedual) {
      if (sensor == 1) {
        SERIALCONSOLE.print("Low Range ");
      } else {
        SERIALCONSOLE.print("High Range");
      }
    }
    if (settings.cursens == Analoguesing) {
      SERIALCONSOLE.print("Analogue Single ");
    }
    if (settings.cursens == Canbus) {
      SERIALCONSOLE.print("CANbus ");
    }
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA");
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("% SOC ");
    SERIALCONSOLE.print(ampsecond * 0.27777777777778, 2);
    SERIALCONSOLE.print("mAh");
  }
}

void SOCcharged(int y)
{
  if (y == 1)
  {
    SOC = 95;
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
  }
  if (y == 2)
  {
    SOC = 100;
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
  }
}

void Prechargecon()
{
  if (digitalRead(IN1) == HIGH || digitalRead(IN3) == HIGH) //detect Key ON or AC present
  {
    halOutWrite(OUT4, HIGH);//Negative Contactor Close
    contctrl = 2;
    if (Pretimer +  settings.Pretime > millis() || currentact > settings.Precurrent)
    {
      halOutWrite(OUT2, HIGH);//precharge
    }
    else //close main contactor
    {
      halOutWrite(OUT1, HIGH);//Positive Contactor Close
      contctrl = 3;
      if (settings.ChargerDirect == 1)
      {
        bmsstatus = Drive;
      }
       else
      {
        if (digitalRead(IN3) == HIGH)
        {
          bmsstatus = Charge;
        }
        if (digitalRead(IN1) == HIGH)
        {
          bmsstatus = Drive;
        }
      }
      halOutWrite(OUT2, LOW);
    }
  }
  else
  {
    halOutWrite(OUT1, LOW);
    halOutWrite(OUT2, LOW);
    halOutWrite(OUT4, LOW);
    bmsstatus = Ready;
    contctrl = 0;
  }
}

void contcon()
{
  if (contctrl != contstat) //check for contactor request change
  {
    if ((contctrl & 1) == 0)
    {
      analogWrite(OUT5, 0);
      contstat = contstat & 254;
    }
    if ((contctrl & 2) == 0)
    {
      analogWrite(OUT6, 0);
      contstat = contstat & 253;
    }
    if ((contctrl & 4) == 0)
    {
      analogWrite(OUT7, 0);
      contstat = contstat & 251;
    }


    if ((contctrl & 1) == 1)
    {
      if ((contstat & 1) != 1)
      {
        if (conttimer1 == 0)
        {
          analogWrite(OUT5, 255);
          conttimer1 = millis() + pulltime ;
        }
        if (conttimer1 < millis())
        {
          analogWrite(OUT5, settings.conthold);
          contstat = contstat | 1;
          conttimer1 = 0;
        }
      }
    }

    if ((contctrl & 2) == 2)
    {
      if ((contstat & 2) != 2)
      {
        if (conttimer2 == 0)
        {
          if (debug != 0)
          {
            Serial.println();
            Serial.println("pull in OUT6");
          }
          analogWrite(OUT6, 255);
          conttimer2 = millis() + pulltime ;
        }
        if (conttimer2 < millis())
        {
          analogWrite(OUT6, settings.conthold);
          contstat = contstat | 2;
          conttimer2 = 0;
        }
      }
    }
    if ((contctrl & 4) == 4)
    {
      if ((contstat & 4) != 4)
      {
        if (conttimer3 == 0)
        {
          if (debug != 0)
          {
            Serial.println();
            Serial.println("pull in OUT7");
          }
          analogWrite(OUT7, 255);
          conttimer3 = millis() + pulltime ;
        }
        if (conttimer3 < millis())
        {
          analogWrite(OUT7, settings.conthold);
          contstat = contstat | 4;
          conttimer3 = 0;
        }
      }
    }
    /*
       SERIALCONSOLE.print(conttimer);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contctrl);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contstat);
       SERIALCONSOLE.println("  ");
    */

  }
  if (contctrl == 0)
  {
    analogWrite(OUT5, 0);
    analogWrite(OUT6, 0);
  }
}

void calcur()
{
  adc->startContinuous(ACUR1, ADC_0);
  sensor = 1;
  x = 0;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    settings.offset1 = settings.offset1 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  settings.offset1 = settings.offset1 / 21;
  SERIALCONSOLE.print(settings.offset1);
  SERIALCONSOLE.print(" current offset 1 calibrated ");
  SERIALCONSOLE.println("  ");
  x = 0;
  adc->adc0->startContinuous(ACUR2);
  sensor = 2;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    settings.offset2 = settings.offset2 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  settings.offset2 = settings.offset2 / 21;
  SERIALCONSOLE.print(settings.offset2);
  SERIALCONSOLE.print(" current offset 2 calibrated ");
  SERIALCONSOLE.println("  ");
}

/*
  NEW: 8-character name of a cell or sensor for VenusOS (0x374-0x377).
  Format "Pk3 C5" = pack (module) 3, cell 5; "Pk2 T1" = pack 2, temperature sensor 1.
  The pack number is the module address = position in the comm chain.
*/
void canLocationText(uint8_t *buf, uint8_t addr, uint8_t num, char kind)
{
  char text[9];
  memset(buf, 0, 8);
  if (addr == 0) return; // unknown
  snprintf(text, sizeof(text), "Pk%d %c%d", addr, kind, num);
  for (int i = 0; i < 8 && text[i] != 0; i++) buf[i] = text[i];
}

/*
  NEW: firmware version from version.h: VenusOS shows v<FW_YEAR>.<FW_BUILD>.
  FW_BUILD goes up by 1 with every commit and wraps around after 255.
*/
uint8_t fwYear = FW_YEAR;
uint8_t fwWeek = (uint8_t)(FW_BUILD & 0xFF); // name kept: is now the build number
void calcFirmwareDate()
{
  fwYear = FW_YEAR;
  fwWeek = (uint8_t)(FW_BUILD & 0xFF);
}
unsigned long canSlowTimer = 0;

/*
  NEW: one JSON line over USB (Serial), 1x per 5 seconds (JSON_INTERVAL), for Node-RED.
  Only when settings.JsonOut = 1 and the port is open on the PC/GX side.
  Fields:
    lt  duration of the last read round in ms (with many modules: keep it well below 3000)
    hw  Teensy model (T32, T40, T41)
    fw  firmware version          st  bmsstatus (0 Boot,1 Ready,2 Drive,3 Charge,4 Precharge,5 Error)
    cb  CAN bus health: s state (0 OK/error active, 1 error passive, 2 bus off), te/re transmit/receive
        error counter, ef error bits seen in the last 5 s (ESR1: 0x2000 ACK, 0x1000 CRC, 0x800 form,
        0x400 stuff, 0x4000/0x8000 bit, 0x200 TX warning, 0x100 RX warning), ee polls with an error,
        tx/tf sent/refused, rx received, rl lost (receive buffer full), kb speed in kbit/s
    sc/ps  cells in series per string / strings in parallel
    er  1 = the EEPROM was reset to the defaults at this start (e.g. a Teensy 4 flash erases it),
        until a setting is sent again (C, P, K, B); Node-RED then restores the last known settings
    ip  CAN protocol to the inverter (0 Victron, 1 LG RESU LV for SolarEdge RWS)
    ka  LG protocol: seconds since the last 'keep alive' 0x305 of the inverter (-1 = never)
    cx  LG protocol: received CAN messages [id, ext, count, "last data hex"] (max 16 identifiers)
    sx  1 = SOC (and current) from outside over USB ('S' command: Lynx/shunt on the GX via Node-RED)
    sv  1 = SOC calculated from the cell voltage (no current measurement for 30 s or set so)
    sm  storage mode (1 = CVL limited to StoreVsetpoint x Scells, via IN1 when switched on in menu m -> o)
    cvl/ccl/dcl  limits to Victron (V/A)    v pack voltage (V)   i current (A, + = charging)   soc (%)
    so/se  strings online/expected   mo/me modules online/expected   bal number of cells balancing
    lo/hi lowest/highest cell (V) with loid/hiid ("Pk3 C5")   tlo/thi lowest/highest temperature (C)
    ovl overvoltage latch (CCL=0)   mw/ma module warning/alarm
    alm/wrn  the 4 alarm/warning bytes of 0x35A (2 bits per alarm: 1 = active, 2 = OK, 0 = n/a)
    mod  per module: a address, s string (1..), on online, v module voltage, c[6] cells (V),
         t[2] temperatures (C, null = no sensor), b balance mask (bit0 = cell 1)
*/
void jsonFloat(float f, int digits)
{
  if (isnan(f) || isinf(f)) SERIALCONSOLE.print("null");
  else SERIALCONSOLE.print(f, digits);
}

void printJson()
{
  if (!Serial) return; // nobody is listening (port not opened)
  char id[9];
  SERIALCONSOLE.print("{\"fw\":\"");
  SERIALCONSOLE.print(FW_YEAR); SERIALCONSOLE.print("."); SERIALCONSOLE.print(FW_BUILD & 0xFF);
  SERIALCONSOLE.print("\",\"hw\":\""); SERIALCONSOLE.print(BOARD_HW);   // Teensy model: T32, T40, T41
  SERIALCONSOLE.print("\",\"st\":"); SERIALCONSOLE.print(bmsstatus);
  SERIALCONSOLE.print(",\"cvl\":"); jsonFloat(cvlActual, 2);
  SERIALCONSOLE.print(",\"sv\":"); SERIALCONSOLE.print(socFromVoltage ? 1 : 0);   // 1 = SOC from cell voltage (no current measurement)
  SERIALCONSOLE.print(",\"sx\":"); SERIALCONSOLE.print(socExternal ? 1 : 0);      // 1 = SOC from outside (Lynx on the GX via Node-RED)
  SERIALCONSOLE.print(",\"sm\":"); SERIALCONSOLE.print(storagemode);   // 1 = storage mode (CVL = StoreVsetpoint x Scells)
  SERIALCONSOLE.print(",\"ip\":"); SERIALCONSOLE.print(settings.InverterProto);   // 0 Victron, 1 LG RESU LV (SolarEdge)
  SERIALCONSOLE.print(",\"sc\":"); SERIALCONSOLE.print(settings.Scells);          // cells in series per string
  SERIALCONSOLE.print(",\"ps\":"); SERIALCONSOLE.print(settings.Pstrings);        // strings in parallel
  SERIALCONSOLE.print(",\"er\":"); SERIALCONSOLE.print(eepromWasReset ? 1 : 0);   // 1 = settings were reset to the defaults at this start (until set again)
  {
    BmsCanStats cs;
    bmsCan.stats(cs, true);   // error flags: since the previous JSON line (5 s)
    SERIALCONSOLE.print(",\"cb\":{\"s\":"); SERIALCONSOLE.print(cs.state);
    SERIALCONSOLE.print(",\"te\":"); SERIALCONSOLE.print(cs.tec);
    SERIALCONSOLE.print(",\"re\":"); SERIALCONSOLE.print(cs.rec);
    SERIALCONSOLE.print(",\"ef\":"); SERIALCONSOLE.print(cs.errFlags);
    SERIALCONSOLE.print(",\"ee\":"); SERIALCONSOLE.print(cs.errEvents);
    SERIALCONSOLE.print(",\"tx\":"); SERIALCONSOLE.print(cs.txOk);
    SERIALCONSOLE.print(",\"tf\":"); SERIALCONSOLE.print(cs.txFail);
    SERIALCONSOLE.print(",\"rx\":"); SERIALCONSOLE.print(cs.rx);
    SERIALCONSOLE.print(",\"rl\":"); SERIALCONSOLE.print(cs.rxLost);
    SERIALCONSOLE.print(",\"kb\":"); SERIALCONSOLE.print(settings.canSpeed / 1000);
    SERIALCONSOLE.print("}");
  }
  if (settings.InverterProto == 1)
  {
    // keep alive of the inverter: seconds since the last 0x305 (-1 = never)
    SERIALCONSOLE.print(",\"ka\":"); SERIALCONSOLE.print(lgKeepAliveMs ? (long)((millis() - lgKeepAliveMs) / 1000) : -1L);
    SERIALCONSOLE.print(",\"cx\":[");
    char hx[24];
    for (uint8_t i = 0; i < canLogUsed; i++)
    {
      if (i) SERIALCONSOLE.print(",");
      SERIALCONSOLE.print("["); SERIALCONSOLE.print(canLog[i].id); SERIALCONSOLE.print(",");
      SERIALCONSOLE.print(canLog[i].ext); SERIALCONSOLE.print(","); SERIALCONSOLE.print(canLog[i].count); SERIALCONSOLE.print(",\"");
      for (uint8_t k = 0; k < canLog[i].len && k < 8; k++) { snprintf(hx, sizeof(hx), "%02X", canLog[i].buf[k]); SERIALCONSOLE.print(hx); }
      SERIALCONSOLE.print("\"]");
    }
    SERIALCONSOLE.print("]");
  }
  SERIALCONSOLE.print(",\"ccl\":"); jsonFloat(chargecurrent * 0.1f, 1);
  SERIALCONSOLE.print(",\"dcl\":"); jsonFloat(discurrent * 0.1f, 1);
  SERIALCONSOLE.print(",\"v\":"); jsonFloat(bms.getPackVoltage(), 3);
  SERIALCONSOLE.print(",\"i\":"); jsonFloat(currentact / 1000.0f, 1);
  SERIALCONSOLE.print(",\"soc\":"); SERIALCONSOLE.print(SOC);
  SERIALCONSOLE.print(",\"so\":"); SERIALCONSOLE.print(bms.getStringsOnline());
  SERIALCONSOLE.print(",\"se\":"); SERIALCONSOLE.print(bms.getStringsExpected());
  SERIALCONSOLE.print(",\"mo\":"); SERIALCONSOLE.print(bms.getModulesOnline());
  SERIALCONSOLE.print(",\"me\":"); SERIALCONSOLE.print(bms.getExpectedModules());
  SERIALCONSOLE.print(",\"bal\":"); SERIALCONSOLE.print(bms.getBalancing());
  SERIALCONSOLE.print(",\"bv\":"); jsonFloat(settings.balanceVoltage, 3);   // balance from (V)
  SERIALCONSOLE.print(",\"bs\":"); jsonFloat(settings.balanceStart, 3);     // start: difference > (V)
  SERIALCONSOLE.print(",\"bh\":"); jsonFloat(settings.balanceHyst, 3);      // stop: down to difference (V)
  SERIALCONSOLE.print(",\"lt\":"); SERIALCONSOLE.print(lastBlockMs);
  SERIALCONSOLE.print(",\"lo\":"); jsonFloat(bms.getLowCellVolt(), 3);
  canLocationText((uint8_t *)id, bms.lowCellAddr, bms.lowCellNum, 'C'); id[8] = 0;
  SERIALCONSOLE.print(",\"loid\":\""); SERIALCONSOLE.print(id); SERIALCONSOLE.print("\"");
  SERIALCONSOLE.print(",\"hi\":"); jsonFloat(bms.getHighCellVolt(), 3);
  canLocationText((uint8_t *)id, bms.highCellAddr, bms.highCellNum, 'C'); id[8] = 0;
  SERIALCONSOLE.print(",\"hiid\":\""); SERIALCONSOLE.print(id); SERIALCONSOLE.print("\"");
  SERIALCONSOLE.print(",\"tlo\":"); jsonFloat(bms.getLowTemperature(), 1);
  SERIALCONSOLE.print(",\"thi\":"); jsonFloat(bms.getHighTemperature(), 1);
  SERIALCONSOLE.print(",\"ovl\":"); SERIALCONSOLE.print(overVoltLatch ? 1 : 0);
  SERIALCONSOLE.print(",\"mw\":"); SERIALCONSOLE.print(moduleWarning ? 1 : 0);
  SERIALCONSOLE.print(",\"ma\":"); SERIALCONSOLE.print(moduleAlarm ? 1 : 0);
  SERIALCONSOLE.print(",\"alm\":[");
  for (int k = 0; k < 4; k++) { if (k) SERIALCONSOLE.print(","); SERIALCONSOLE.print(alarm[k]); }
  SERIALCONSOLE.print("],\"wrn\":[");
  for (int k = 0; k < 4; k++) { if (k) SERIALCONSOLE.print(","); SERIALCONSOLE.print(warning[k]); }
  SERIALCONSOLE.print("],\"mod\":[");
  bool first = true;
  for (int a = 1; a <= MAX_MODULE_ADDR; a++)
  {
    if (!bms.moduleExists(a)) continue;
    if (!first) SERIALCONSOLE.print(",");
    first = false;
    SERIALCONSOLE.print("{\"a\":"); SERIALCONSOLE.print(a);
    SERIALCONSOLE.print(",\"s\":");
    if (bms.isConfiguredModule(a)) SERIALCONSOLE.print(bms.stringOfModule(a) + 1); else SERIALCONSOLE.print(0);
    SERIALCONSOLE.print(",\"on\":"); SERIALCONSOLE.print(bms.isModuleOnline(a) ? 1 : 0);
    SERIALCONSOLE.print(",\"v\":"); jsonFloat(bms.getModuleVolt(a), 3);
    SERIALCONSOLE.print(",\"c\":[");
    for (int c = 0; c < 6; c++) { if (c) SERIALCONSOLE.print(","); jsonFloat(bms.getCellV(a, c), 3); }
    SERIALCONSOLE.print("],\"t\":[");
    for (int t = 0; t < 2; t++)
    {
      if (t) SERIALCONSOLE.print(",");
      float tc = bms.getModuleTemp(a, t);
      if (tc <= -70.0f) SERIALCONSOLE.print("null"); else jsonFloat(tc, 1);
    }
    SERIALCONSOLE.print("],\"b\":"); SERIALCONSOLE.print(bms.getBalanceMask(a));
    SERIALCONSOLE.print("}");
  }
  SERIALCONSOLE.println("]}");
}

/*
  NEW: LG RESU LV protocol (CAN 500 kbit/s, 11-bit), for inverters that support the LG Chem
  RESU 3.3 - 13 low voltage batteries, e.g. SolarEdge StorEdge SE5K-SE10K-RWS (40-62 V).
  Messages of a real LG RESU LV (reverse engineered, see github.com/jens18/lgresu):
    0x351  charge voltage (0.1 V), CCL (0.1 A), DCL (0.1 A)            little endian
    0x355  SOC (%), SOH (%)
    0x356  voltage (0.01 V), current (0.1 A, + = charging), temperature (0.1 C)
    0x359  warnings (16 bits) + alarms (16 bits) in LG layout
  The inverter sends 0x305 ('keep alive'). Everything the inverter sends is logged (JSON "cx").
  Charging/discharging is allowed or blocked with CCL/DCL; the inverter itself decides the power
  (SolarEdge: storage profile / meter / Modbus remote control).
*/
void lgResuCan()
{
  if (cvlActual <= 0.0f) cvlControl();
  msg.ext = 0;

  msg.id  = 0x351;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(cvlActual * 10));
  msg.buf[1] = highByte(uint16_t(cvlActual * 10));
  msg.buf[2] = lowByte(chargecurrent);
  msg.buf[3] = highByte(chargecurrent);
  msg.buf[4] = lowByte(discurrent);
  msg.buf[5] = highByte(discurrent);
  msg.buf[6] = lowByte(uint16_t((dischargeCellV() * settings.Scells) * 10));
  msg.buf[7] = highByte(uint16_t((dischargeCellV() * settings.Scells) * 10));
  bmsCan.write(msg);
  delay(2);

  msg.id  = 0x355;
  msg.len = 8;
  memset(msg.buf, 0, 8);
  msg.buf[0] = lowByte(uint16_t(SOC));
  msg.buf[1] = highByte(uint16_t(SOC));
  msg.buf[2] = lowByte(uint16_t(SOH));
  msg.buf[3] = highByte(uint16_t(SOH));
  bmsCan.write(msg);
  delay(2);

  msg.id  = 0x356;
  msg.len = 8;
  memset(msg.buf, 0, 8);
  msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[2] = lowByte(int16_t(currentact / 100));
  msg.buf[3] = highByte(int16_t(currentact / 100));
  msg.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
  bmsCan.write(msg);
  delay(2);

  // 0x359 warnings in the LG layout (bit numbers of the 16-bit little endian word)
  float hc = bms.getHighCellVolt(), lc = bms.getLowCellVolt();
  uint16_t w = 0;
  if (hc >= settings.OverVSetpoint) w |= 0x0002;                                   // battery high voltage
  if (lc < settings.UnderVSetpoint) w |= 0x0004;                                    // battery low voltage
  if (bms.getHighTemperature() > settings.OverTSetpoint - settings.WarnToff) w |= 0x0008; // high temperature
  if (bms.getLowTemperature() < settings.UnderTSetpoint + settings.WarnToff) w |= 0x0010; // low temperature
  if (moduleWarning || bms.getStringsOnline() == 0) w |= 0x0800;                    // BMS internal
  if ((hc - lc) > settings.CellGap) w |= 0x1000;                                     // cell imbalance
  msg.id  = 0x359;
  msg.len = 8;
  memset(msg.buf, 0, 8);
  msg.buf[0] = lowByte(w);
  msg.buf[1] = highByte(w);
  // bytes 2-3: alarms (meaning of the bits unknown): 0
  bmsCan.write(msg);
  delay(2);
}

void VEcan() //communication with Victron system over CAN
{
  msg.id  = 0x351;
  msg.len = 8;
  // NEW: CVL from cvlControl() (0.1 V), lowered in a controlled way when a cell runs ahead
  if (cvlActual <= 0.0f) cvlControl();
  msg.buf[0] = lowByte(uint16_t(cvlActual * 10));
  msg.buf[1] = highByte(uint16_t(cvlActual * 10));
  msg.buf[2] = lowByte(chargecurrent);
  msg.buf[3] = highByte(chargecurrent);
  msg.buf[4] = lowByte(discurrent );
  msg.buf[5] = highByte(discurrent);
  msg.buf[6] = lowByte(uint16_t((dischargeCellV() * settings.Scells) * 10));
  msg.buf[7] = highByte(uint16_t((dischargeCellV() * settings.Scells) * 10));
  bmsCan.write(msg);

  msg.id  = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(SOC);
  msg.buf[1] = highByte(SOC);
  msg.buf[2] = lowByte(SOH);
  msg.buf[3] = highByte(SOH);
  msg.buf[4] = lowByte(uint16_t(SOC * 100)); // NEW: 0.01% resolution (was SOC*10)
  msg.buf[5] = highByte(uint16_t(SOC * 100));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  bmsCan.write(msg);

  msg.id  = 0x356;
  msg.len = 8;

  if (settings.chargertype == VictronHV || settings.SerialCan == 1)
  {
    msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 10));
    msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 10));
  }
  else
  {
    msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
    msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  }

  msg.buf[2] = lowByte(long(currentact / 100));
  msg.buf[3] = highByte(long(currentact / 100));
  msg.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  bmsCan.write(msg);


  delay(2);
  msg.id  = 0x35A;
  msg.len = 8;
  msg.buf[0] = alarm[0];//High temp  Low Voltage | High Voltage
  msg.buf[1] = alarm[1]; // High Discharge Current | Low Temperature
  msg.buf[2] = alarm[2]; //Internal Failure | High Charge current
  msg.buf[3] = alarm[3];// Cell Imbalance
  msg.buf[4] = warning[0];//High temp  Low Voltage | High Voltage
  msg.buf[5] = warning[1];// High Discharge Current | Low Temperature
  msg.buf[6] = warning[2];//Internal Failure | High Charge current
  msg.buf[7] = warning[3];// Cell Imbalance
  bmsCan.write(msg);

  msg.id  = 0x35E;
  msg.len = 8;
  msg.buf[0] = bmsname[0];
  msg.buf[1] = bmsname[1];
  msg.buf[2] = bmsname[2];
  msg.buf[3] = bmsname[3];
  msg.buf[4] = bmsname[4];
  msg.buf[5] = bmsname[5];
  msg.buf[6] = bmsname[6];
  msg.buf[7] = bmsname[7];
  bmsCan.write(msg);

  delay(2);
  // battery name part 1 (characters 1-8)
  msg.id  = 0x370;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = batteryName[i];
  bmsCan.write(msg);

  delay(2);
  // battery name part 2 (characters 9-16)
  msg.id  = 0x371;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = batteryName[8 + i];
  bmsCan.write(msg);

  delay(2);
  msg.id  = 0x373;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
  msg.buf[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
  msg.buf[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
  msg.buf[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
  msg.buf[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
  msg.buf[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
  msg.buf[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
  msg.buf[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
  bmsCan.write(msg);

  delay(2);
  msg.id  = 0x379; //Installed capacity
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(settings.Pstrings * settings.CAP));
  msg.buf[1] = highByte(uint16_t(settings.Pstrings * settings.CAP));
  msg.buf[2] = contstat; //contactor state
  msg.buf[3] = (halOutRead(OUT1) | (halOutRead(OUT2) << 1) | (halOutRead(OUT3) << 2) | (halOutRead(OUT4) << 3));
  msg.buf[4] = bmsstatus;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  bmsCan.write(msg);
  /*
      delay(2);
    msg.id  = 0x378; //Installed capacity
    msg.len = 2;
    //energy in 100wh/unit
    msg.buf[0] =
    msg.buf[1] =
    msg.buf[2] =
    msg.buf[3] =
    //energy out 100wh/unit
    msg.buf[4] =
    msg.buf[5] =
    msg.buf[6] =
    msg.buf[7] =
  */
  delay(2);

  // NEW: once per second: battery info + location of min/max cell and temperature
  if (millis() - canSlowTimer >= 1000)
  {
    canSlowTimer = millis();
    if (fwYear == 0) calcFirmwareDate();

    msg.id  = 0x35F; // battery model / firmware / available capacity
    msg.len = 8;
    msg.buf[0] = 0x00;           // battery model (not used by Victron)
    msg.buf[1] = 0x00;
    msg.buf[2] = fwYear;         // firmware: year (version.h)
    msg.buf[3] = fwWeek;         // firmware: build number (version.h)
    msg.buf[4] = lowByte(uint16_t(settings.Pstrings * settings.CAP));  // capacity Ah
    msg.buf[5] = highByte(uint16_t(settings.Pstrings * settings.CAP));
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    bmsCan.write(msg);
    delay(2);

    msg.id = 0x374; msg.len = 8; canLocationText(msg.buf, bms.lowCellAddr, bms.lowCellNum, 'C');  bmsCan.write(msg); delay(2);
    msg.id = 0x375; msg.len = 8; canLocationText(msg.buf, bms.highCellAddr, bms.highCellNum, 'C'); bmsCan.write(msg); delay(2);
    msg.id = 0x376; msg.len = 8; canLocationText(msg.buf, bms.lowTempAddr, bms.lowTempNum, 'T');  bmsCan.write(msg); delay(2);
    msg.id = 0x377; msg.len = 8; canLocationText(msg.buf, bms.highTempAddr, bms.highTempNum, 'T'); bmsCan.write(msg); delay(2);
  }

  msg.id  = 0x372;
  msg.len = 8;
  // VenusOS "modules online/offline": the PHYSICAL number of Tesla modules (Anco 2026-11-14),
  // regardless of 24 V or 48 V. (The control itself works per string.)
  {
    uint16_t modOn = bms.getModulesOnline();
    uint16_t modExp = bms.getExpectedModules();
    uint16_t modOff = (modExp > modOn) ? (modExp - modOn) : 0;
    msg.buf[0] = lowByte(modOn);                                   // modules online
    msg.buf[1] = highByte(modOn);
    msg.buf[2] = lowByte(uint16_t(chargecurrent == 0 ? modOn : 0)); // blocking charge
    msg.buf[3] = highByte(uint16_t(chargecurrent == 0 ? modOn : 0));
    msg.buf[4] = lowByte(uint16_t(discurrent == 0 ? modOn : 0));    // blocking discharge
    msg.buf[5] = highByte(uint16_t(discurrent == 0 ? modOn : 0));
    msg.buf[6] = lowByte(modOff);                                  // modules offline
    msg.buf[7] = highByte(modOff);
  }
  bmsCan.write(msg);

}

// Settings menu
void menu()
{

  incomingByte = Serial.read(); // read the incoming byte:
  if (menuload == 4)
  {
    switch (incomingByte)
    {

      case '1':
        menuload = 1;
        candebug = !candebug;
        incomingByte = 'd';
        break;

      case '2':
        menuload = 1;
        debugCur = !debugCur;
        incomingByte = 'd';
        break;

      case '3':
        menuload = 1;
        outputcheck = !outputcheck;
        if (outputcheck == 0)
        {
          contctrl = 0;
          halOutWrite(OUT1, LOW);
          halOutWrite(OUT2, LOW);
          halOutWrite(OUT3, LOW);
          halOutWrite(OUT4, LOW);
        }
        incomingByte = 'd';
        break;

      case '4':
        menuload = 1;
        inputcheck = !inputcheck;
        incomingByte = 'd';
        break;

      case '5':
        menuload = 1;
        settings.ESSmode = !settings.ESSmode;
        incomingByte = 'd';
        break;

      case '6':
        menuload = 1;
        cellspresent = bms.seriescells();
        incomingByte = 'd';
        break;

      case '7':
        menuload = 1;
        gaugedebug = !gaugedebug;
        incomingByte = 'd';
        break;

      case '8':
        menuload = 1;
        CSVdebug = !CSVdebug;
        incomingByte = 'd';
        break;

      case '9':
        menuload = 1;
        if (Serial.available() > 0)
        {
          debugdigits = Serial.parseInt();
        }
        if (debugdigits > 4)
        {
          debugdigits = 2;
        }
        incomingByte = 'd';
        break;

      case '0':
        menuload = 1;
        CPdebug = !CPdebug;
        incomingByte = 'd';
        break;

      case 'd':
        menuload = 1;
        if (Serial.available() > 0)
        {
          delim = Serial.parseInt();
        }
        if (delim > 1)
        {
          delim = 0;
        }
        incomingByte = 'd';
        break;

      case 113: //q for quite menu

        menuload = 0;
        incomingByte = 115;
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (menuload == 10) // NEW: modules & strings
  {
    switch (incomingByte)
    {
      case 'f': // search again (keep addresses)
        SERIALCONSOLE.println("Searching for modules...");
        bms.findBoards();
        bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
        menuload = 1;
        incomingByte = 'm';
        break;

      case 'r': // renumber: assign addresses again in chain order
        SERIALCONSOLE.println("Renumbering all modules in chain order...");
        if (settings.AutoTopology == 1)
        {
          autoDetectTopology(); // also adopt the new number
        }
        else
        {
          bms.renumberBoardIDs();
          bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
        }
        bms.clearFaults();
        menuload = 1;
        incomingByte = 'm';
        break;

      case 't': // alarm delay in s
        if (Serial.available() > 0)
        {
          settings.ModuleAlarmDelay = Serial.parseInt();
          menuload = 1;
          incomingByte = 'm';
        }
        break;

      case 'a': // auto-detect at startup on/off
        settings.AutoTopology = !settings.AutoTopology;
        menuload = 1;
        incomingByte = 'm';
        break;

      case 'j': // JSON output over USB (Node-RED) on/off
        settings.JsonOut = !settings.JsonOut;
        menuload = 1;
        incomingByte = 'm';
        break;

      case 'p': // CAN protocol to the inverter: Victron <-> LG RESU LV (SolarEdge)
        settings.InverterProto = settings.InverterProto ? 0 : 1;
        canLogUsed = 0;
        menuload = 1;
        incomingByte = 'm';
        break;

      case 'o': // storage mode via input IN1 on/off
        settings.StorageInput = !settings.StorageInput;
        menuload = 1;
        incomingByte = 'm';
        break;

      case 'v': // CVL control on/off
        settings.CVLControl = !settings.CVLControl;
        menuload = 1;
        incomingByte = 'm';
        break;

      case 's': // scaling CCL/DCL on/off
        settings.ScaleLimits = !settings.ScaleLimits;
        menuload = 1;
        incomingByte = 'm';
        break;

      case 113: //q to go back to main menu
        menuload = 0;
        incomingByte = 115;
        break;
    }
  }

  if (menuload == 9)
  {
    if (settings.ExpMess > 1)
    {
      settings.ExpMess = 0;
    }
    switch (incomingByte)
    {

      case '1':
        menuload = 1;
        settings.ExpMess = !settings.ExpMess;
        incomingByte = 'x';
        break;
      case 113: //q to go back to main menu
        menuload = 0;
        incomingByte = 115;
        break;
    }
  }

  if (menuload == 2)
  {
    switch (incomingByte)
    {


      case 99: //c for calibrate zero offset

        calcur();
        break;

      case '1':
        menuload = 1;
        settings.invertcur = !settings.invertcur;
        incomingByte = 'c';
        break;

      case '2':
        menuload = 1;
        settings.voltsoc = !settings.voltsoc;
        incomingByte = 'c';
        break;

      case '3':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.ncur = Serial.parseInt();
        }
        menuload = 1;
        incomingByte = 'c';
        break;

      case '8':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.changecur = Serial.parseInt();
        }
        menuload = 1;
        incomingByte = 'c';
        break;

      case '4':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.convlow = Serial.parseInt();
        }
        incomingByte = 'c';
        break;

      case '5':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.convhigh = Serial.parseInt();
        }
        incomingByte = 'c';
        break;

      case '6':
        menuload = 1;
        if (Serial.available() > 0)
        {
          settings.CurDead = Serial.parseInt();
        }
        incomingByte = 'c';
        break;

      case 113: //q for quite menu

        menuload = 0;
        incomingByte = 115;
        break;

      case 115: //s for switch sensor
        settings.cursens ++;
        if (settings.cursens > 3)
        {
          settings.cursens = 0;
        }
        /*
          if (settings.cursens == Analoguedual)
          {
            settings.cursens = Canbus;
            SERIALCONSOLE.println("  ");
            SERIALCONSOLE.print(" CANbus Current Sensor ");
            SERIALCONSOLE.println("  ");
          }
          else
          {
            settings.cursens = Analoguedual;
            SERIALCONSOLE.println("  ");
            SERIALCONSOLE.print(" Analogue Current Sensor ");
            SERIALCONSOLE.println("  ");
          }
        */
        menuload = 1;
        incomingByte = 'c';
        break;

      case '7': //s for switch sensor
        settings.curcan++;
        if (settings.curcan > CurCanMax) {
          settings.curcan = 1;
        }
        menuload = 1;
        incomingByte = 'c';
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (menuload == 8)
  {
    switch (incomingByte)
    {
      case '1': //e dispaly settings
        if (Serial.available() > 0)
        {
          settings.IgnoreTemp = Serial.parseInt();
        }
        if (settings.IgnoreTemp > 2)
        {
          settings.IgnoreTemp = 0;
        }
        bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
        menuload = 1;
        incomingByte = 'i';
        break;

      case '2':
        if (Serial.available() > 0)
        {
          settings.IgnoreVolt = Serial.parseInt();
          settings.IgnoreVolt = settings.IgnoreVolt * 0.001;
          bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
          // Serial.println(settings.IgnoreVolt);
          menuload = 1;
          incomingByte = 'i';
        }
        break;

      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115;
        break;
    }
  }



  if (menuload == 7)
  {
    switch (incomingByte)
    {
      case '1':
        if (Serial.available() > 0)
        {
          settings.WarnOff = Serial.parseInt();
          settings.WarnOff = settings.WarnOff * 0.001;
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '2':
        if (Serial.available() > 0)
        {
          settings.CellGap = Serial.parseInt();
          settings.CellGap = settings.CellGap * 0.001;
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '3':
        if (Serial.available() > 0)
        {
          settings.WarnToff = Serial.parseInt();
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case '4':
        if (Serial.available() > 0)
        {
          settings.triptime = Serial.parseInt();
          menuload = 1;
          incomingByte = 'a';
        }
        break;

      case 113: //q to go back to main menu
        menuload = 0;
        incomingByte = 115;
        break;
    }
  }

  if (menuload == 6) //Charging settings
  {
    switch (incomingByte)
    {

      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115;
        break;

      case '1':
        if (Serial.available() > 0)
        {
          settings.ChargeVsetpoint = Serial.parseInt();
          settings.ChargeVsetpoint = settings.ChargeVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'e';
        }
        break;


      case '2':
        if (Serial.available() > 0)
        {
          settings.ChargeHys = Serial.parseInt();
          settings.ChargeHys = settings.ChargeHys / 1000;
          menuload = 1;
          incomingByte = 'e';
        }
        break;


      case '4':
        if (Serial.available() > 0)
        {
          settings.chargecurrentend = Serial.parseInt() * 10;
          menuload = 1;
          incomingByte = 'e';
        }
        break;


      case '3':
        if (Serial.available() > 0)
        {
          settings.chargecurrentmax = Serial.parseInt() * 10;
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case 'a':
        if (Serial.available() > 0)
        {
          settings.chargecurrent2max = Serial.parseInt() * 10;
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case '5': //1 Over Voltage Setpoint
        settings.chargertype = settings.chargertype + 1;
        if (settings.chargertype > 7)
        {
          settings.chargertype = 0;
        }
        menuload = 1;
        incomingByte = 'e';
        break;

      case '6':
        if (Serial.available() > 0)
        {
          settings.chargerspd = Serial.parseInt();
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case '7':
        if (Serial.available() > 0)
        {
          settings.canSpeed = Serial.parseInt() * 1000;
          bmsCan.end();
          bmsCan.begin(settings.canSpeed);
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case '8':
        if ( settings.ChargerDirect == 1)
        {
          settings.ChargerDirect = 0;
          menuload = 1;
          incomingByte = 'e';
        }
        else
        {
          settings.ChargerDirect = 1;
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case '9':
        if (Serial.available() > 0)
        {
          settings.ChargeTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case 'b':
        if (Serial.available() > 0)
        {
          settings.chargereff = Serial.parseInt();
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case 'c':
        if (Serial.available() > 0)
        {
          settings.chargerACv = Serial.parseInt();
          menuload = 1;
          incomingByte = 'e';
        }
        break;

      case 'd':
        if (Serial.available() > 0)
        {
          if (settings.SerialCan == 0)
          {
            settings.SerialCan = 1;
          }
          else
          {
            settings.SerialCan = 0;
          }
          menuload = 1;
          incomingByte = 'e';
        }
        break;

    }
  }

  if (menuload == 5)
  {
    switch (incomingByte)
    {
      case '1':
        if (Serial.available() > 0)
        {
          settings.Pretime = Serial.parseInt();
          menuload = 1;
          incomingByte = 'k';
        }
        break;

      case '2':
        if (Serial.available() > 0)
        {
          settings.Precurrent = Serial.parseInt();
          menuload = 1;
          incomingByte = 'k';
        }
        break;

      case '3':
        if (Serial.available() > 0)
        {
          settings.conthold = Serial.parseInt();
          menuload = 1;
          incomingByte = 'k';
        }
        break;

      case '4':
        if (Serial.available() > 0)
        {
          settings.gaugelow = Serial.parseInt();
          gaugedebug = 2;
          gaugeupdate();
          menuload = 1;
          incomingByte = 'k';
        }
        break;

      case '5':
        if (Serial.available() > 0)
        {
          settings.gaugehigh = Serial.parseInt();
          gaugedebug = 3;
          gaugeupdate();
          menuload = 1;
          incomingByte = 'k';
        }
        break;

      case '6':
        settings.tripcont = !settings.tripcont;
        if (settings.tripcont > 1)
        {
          settings.tripcont = 0;
        }
        menuload = 1;
        incomingByte = 'k';
        break;


      case '7':
        if ( settings.ChargerDirect == 1)
        {
          settings.ChargerDirect = 0;
        }
        else
        {
          settings.ChargerDirect = 1;
        }
        menuload = 1;
        incomingByte = 'k';
        break;

      case 113: //q to go back to main menu
        gaugedebug = 0;
        menuload = 0;
        incomingByte = 115;
        break;
    }
  }

  if (menuload == 3)
  {
    switch (incomingByte)
    {
      case 113: //q to go back to main menu

        menuload = 0;
        incomingByte = 115;
        break;

      case 'b':
        if (Serial.available() > 0)
        {
          settings.socvolt[0] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'f': //f factory settings
        loadSettings();
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println(" Coded Settings Loaded ");
        SERIALCONSOLE.println("  ");
        menuload = 1;
        incomingByte = 'b';
        break;

case 'r':  //r for reset
        SOCreset = 1;
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(" mAh Reset ");
        SERIALCONSOLE.println("  ");
        menuload = 1;
        incomingByte = 'b';
        break;


      case '1': //1 Over Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.OverVSetpoint = Serial.parseInt();
          settings.OverVSetpoint = settings.OverVSetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'g':
        if (Serial.available() > 0)
        {
          settings.StoreVsetpoint = Serial.parseInt();
          settings.StoreVsetpoint = settings.StoreVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'h':
        if (Serial.available() > 0)
        {
          settings.DisTaper = Serial.parseInt();
          settings.DisTaper = settings.DisTaper / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'j':
        if (Serial.available() > 0)
        {
          settings.DisTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'c':
        if (Serial.available() > 0)
        {
          settings.socvolt[1] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'd':
        if (Serial.available() > 0)
        {
          settings.socvolt[2] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'e':
        if (Serial.available() > 0)
        {
          settings.socvolt[3] = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '9': //Discharge Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.DischVsetpoint = Serial.parseInt();
          settings.DischVsetpoint = settings.DischVsetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'k': //Discharge Voltage hysteresis
        if (Serial.available() > 0)
        {
          settings.DischHys = Serial.parseInt();
          settings.DischHys  = settings.DischHys  / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '0': //c Pstrings
        if (Serial.available() > 0)
        {
          settings.Pstrings = Serial.parseInt();
          if (settings.Pstrings < 1) settings.Pstrings = 1;
          menuload = 1;
          incomingByte = 'b';
          if (settings.AutoTopology == 1)
          {
            SERIALCONSOLE.println("Note: auto-detect is on, the number of strings follows from the modules found.");
            SERIALCONSOLE.println("      For a fixed number: switch auto-detect off in menu m -> a.");
          }
          applyTopology();
        }
        break;

      case 'a': //
        if (Serial.available() > 0)
        {
          settings.Scells  = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
          applyTopology();   // with auto-detect: number of strings is recalculated immediately
        }
        break;

      case '2': //2 Under Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderVSetpoint = Serial.parseInt();
          settings.UnderVSetpoint =  settings.UnderVSetpoint / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '3': //3 Over Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.OverTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '4': //4 Udner Temperature Setpoint
        if (Serial.available() > 0)
        {
          settings.UnderTSetpoint = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '5': //5 Balance Voltage Setpoint
        if (Serial.available() > 0)
        {
          settings.balanceVoltage = Serial.parseInt();
          settings.balanceVoltage = settings.balanceVoltage / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '6': //6 Balance STOP difference (mV)
        if (Serial.available() > 0)
        {
          settings.balanceHyst = Serial.parseInt();
          settings.balanceHyst =  settings.balanceHyst / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case 'l': // NEW: Balance START difference (mV)
        if (Serial.available() > 0)
        {
          settings.balanceStart = Serial.parseInt();
          settings.balanceStart = settings.balanceStart / 1000;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '7'://7 Battery Capacity inAh
        if (Serial.available() > 0)
        {
          settings.CAP = Serial.parseInt();
          menuload = 1;
          incomingByte = 'b';
        }
        break;

      case '8':// discurrent in A
        if (Serial.available() > 0)
        {
          settings.discurrentmax = Serial.parseInt() * 10;
          menuload = 1;
          incomingByte = 'b';
        }
        break;

    }
  }

  if (menuload == 1)
  {
    switch (incomingByte)
    {
      case 'R'://restart
        CPU_REBOOT ;
        break;

      case 'm': // NEW: modules & strings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Modules & Strings Menu");
        bms.printTopology();
        SERIALCONSOLE.print("a - Auto-detect at startup (count modules = expected number) : ");
        SERIALCONSOLE.println(settings.AutoTopology ? "On" : "Off (fixed number via menu b -> 0)");
        SERIALCONSOLE.println("f - Search modules again (keep addresses)");
        SERIALCONSOLE.println("r - Renumber modules in chain order (string 1 first!), with auto: count again too");
        SERIALCONSOLE.print("t - Delay warning -> alarm when a string is missing : ");
        SERIALCONSOLE.print(settings.ModuleAlarmDelay);
        SERIALCONSOLE.println(" s");
        SERIALCONSOLE.print("j - JSON output over USB for Node-RED (1x per 5 s, text off) : ");
        SERIALCONSOLE.println(settings.JsonOut ? "On" : "Off");
        SERIALCONSOLE.print("v - Victron CVL control (lower CVL 0.1V/s, CCL=0 only at OverV) : ");
        SERIALCONSOLE.println(settings.CVLControl ? "On" : "Off (old: CCL taper, fixed CVL)");
        SERIALCONSOLE.print("p - CAN protocol to the inverter : ");
        SERIALCONSOLE.println(settings.InverterProto ? "LG RESU LV (SolarEdge StorEdge RWS)" : "Victron");
        SERIALCONSOLE.print("o - Storage mode via input IN1 (CVL = StoreVsetpoint x Scells) : ");
        SERIALCONSOLE.println(settings.StorageInput ? "On" : "Off");
        SERIALCONSOLE.print("s - Scale CCL/DCL with strings online : ");
        SERIALCONSOLE.println(settings.ScaleLimits ? "On" : "Off");
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 10;
        break;

      case 'x': //Ignore Value Settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Experimental Settings");
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Do not use unless you know what it does!!!!!");
        SERIALCONSOLE.print("1 - Sending Experimental Victron CAN:");
        SERIALCONSOLE.println(settings.ExpMess);

        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 9;
        break;

      case 'i': //Ignore Value Settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Ignore Value Settings");
        SERIALCONSOLE.print("1 - Temp Sensor Setting:");
        SERIALCONSOLE.println(settings.IgnoreTemp);
        SERIALCONSOLE.print("2 - Voltage Under Which To Ignore Cells:");
        SERIALCONSOLE.print(settings.IgnoreVolt * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 8;
        break;

      case 'e': //Charging settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Charging Settings");
        SERIALCONSOLE.print("1 - Cell Charge Voltage Limit Setpoint: ");
        SERIALCONSOLE.print(settings.ChargeVsetpoint * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("2 - Charge Hystersis: ");
        SERIALCONSOLE.print(settings.ChargeHys * 1000, 0 );
        SERIALCONSOLE.println("mV");
        // always shown: also used for the CCL towards Victron
        SERIALCONSOLE.print("3 - Max Charge Current PER STRING: ");
        SERIALCONSOLE.print(settings.chargecurrentmax * 0.1);
        SERIALCONSOLE.println("A");
        SERIALCONSOLE.print("4 - End of Charge Current PER STRING: ");
        SERIALCONSOLE.print(settings.chargecurrentend * 0.1);
        SERIALCONSOLE.println("A");
        SERIALCONSOLE.print("5- Charger Type: ");
        switch (settings.chargertype)
        {
          case 0:
            SERIALCONSOLE.print("Relay Control");
            break;
          case 1:
            SERIALCONSOLE.print("Brusa NLG5xx");
            break;
          case 2:
            SERIALCONSOLE.print("Volt Charger");
            break;
          case 3:
            SERIALCONSOLE.print("Eltek Charger");
            break;
          case 4:
            SERIALCONSOLE.print("Elcon Charger");
            break;
          case 5:
            SERIALCONSOLE.print("Victron/SMA");
            break;
          case 6:
            SERIALCONSOLE.print("Coda");
            break;
          case 7:
            SERIALCONSOLE.print("Victron HV Spec");
            break;
        }
        SERIALCONSOLE.println();
        if (settings.chargertype > 0)
        {
          SERIALCONSOLE.print("6- Charger Can Msg Spd: ");
          SERIALCONSOLE.print(settings.chargerspd);
          SERIALCONSOLE.println("mS");
          SERIALCONSOLE.print("7- Can Baudrate: ");
          SERIALCONSOLE.print(settings.canSpeed * 0.001, 0);
          SERIALCONSOLE.println("kbps");
        }
        SERIALCONSOLE.print("8 - Charger HV Connection: ");
        switch (settings.ChargerDirect)
        {
          case 0:
            SERIALCONSOLE.print(" Behind Contactors");
            break;
          case 1:
            SERIALCONSOLE.print("Direct To Battery HV");
            break;
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("9 - Charge Current derate Low: ");
        SERIALCONSOLE.print(settings.ChargeTSetpoint);
        SERIALCONSOLE.println(" C");
        if (settings.chargertype > 0)
        {
          SERIALCONSOLE.print("a - Alternate Max Charge Current PER STRING: ");
          SERIALCONSOLE.print(settings.chargecurrent2max * 0.1);
          SERIALCONSOLE.println("A");
          SERIALCONSOLE.print("b - Charger AC to DC effiecency: ");
          SERIALCONSOLE.print(settings.chargereff);
          SERIALCONSOLE.println("%");
          SERIALCONSOLE.print("c - Charger AC Voltage: ");
          SERIALCONSOLE.print(settings.chargerACv);
          SERIALCONSOLE.println("VAC");
        }

        SERIALCONSOLE.print("d - Standard Can Voltage Scale: ");
        if (settings.SerialCan == 0)
        {
          SERIALCONSOLE.print("0.01");
        }
        else if (settings.SerialCan == 1)
        {
          SERIALCONSOLE.print("0.1");
        }
        SERIALCONSOLE.println();

        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 6;
        break;

      case 'a': //Alarm and Warning settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Alarm and Warning Settings Menu");
        SERIALCONSOLE.print("1 - Voltage Warning Offset (not used anymore): ");
        SERIALCONSOLE.print(settings.WarnOff * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("2 - Cell Voltage Difference Alarm: ");
        SERIALCONSOLE.print(settings.CellGap * 1000, 0);
        SERIALCONSOLE.println("mV");
        SERIALCONSOLE.print("3 - Temp Warning Offset: ");
        SERIALCONSOLE.print(settings.WarnToff);
        SERIALCONSOLE.println(" C");
        //SERIALCONSOLE.print("4 - Temp Warning delay: ");
        //SERIALCONSOLE.print(settings.UnderDur);
        //SERIALCONSOLE.println(" mS");
        SERIALCONSOLE.print("4 - Over and Under Voltage Delay: ");
        SERIALCONSOLE.print(settings.triptime);
        SERIALCONSOLE.println(" mS");

        menuload = 7;
        break;

      case 'k': //contactor settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Contactor and Gauge Settings Menu");
        SERIALCONSOLE.print("1 - PreCharge Timer: ");
        SERIALCONSOLE.print(settings.Pretime);
        SERIALCONSOLE.println("mS");
        SERIALCONSOLE.print("2 - PreCharge Finish Current: ");
        SERIALCONSOLE.print(settings.Precurrent);
        SERIALCONSOLE.println(" mA");
        SERIALCONSOLE.print("3 - PWM contactor Hold 0 - 255 : ");
        SERIALCONSOLE.println(settings.conthold);
        SERIALCONSOLE.print("4 - PWM for Gauge Low 0 - 255 : ");
        SERIALCONSOLE.println(settings.gaugelow);
        SERIALCONSOLE.print("5 - PWM for Gauge High 0 - 255 : ");
        SERIALCONSOLE.println(settings.gaugehigh);
        if (settings.ESSmode == 1)
        {
          SERIALCONSOLE.print("6 - ESS Main Contactor or Trip : ");
          if (settings.tripcont == 0)
          {
            SERIALCONSOLE.println( "Trip Shunt");
          }
          else
          {
            SERIALCONSOLE.println( "Main Contactor and Precharge");
          }
          SERIALCONSOLE.print("7 - External Battery Enable : ");
          switch (settings.ChargerDirect)
          {
            case 0:
              SERIALCONSOLE.print(" Enable In2");
              break;
            case 1:
              SERIALCONSOLE.print("Auto Start");
              break;
          }
        }


        menuload = 5;
        break;

      case 113: //q to go back to main menu
        EEPROM.put(0, settings); //save all change to eeprom
        menuload = 0;
        debug = (settings.JsonOut == 1) ? 0 : 1; // JSON mode: keep the text output off
        break;
      case 'd': //d for debug settings
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Debug Settings Menu");
        SERIALCONSOLE.println("Toggle on / off");
        SERIALCONSOLE.print("1 - Can Debug : ");
        SERIALCONSOLE.println(candebug);
        SERIALCONSOLE.print("2 - Current Debug : ");
        SERIALCONSOLE.println(debugCur);
        SERIALCONSOLE.print("3 - Output Check : ");
        SERIALCONSOLE.println(outputcheck);
        SERIALCONSOLE.print("4 - Input Check : ");
        SERIALCONSOLE.println(inputcheck);
        SERIALCONSOLE.print("5 - ESS mode : ");
        SERIALCONSOLE.println(settings.ESSmode);
        SERIALCONSOLE.print("6 - Cells Present Reset : ");
        SERIALCONSOLE.println(cellspresent);
        SERIALCONSOLE.print("7 - Gauge Debug : ");
        SERIALCONSOLE.println(gaugedebug);
        SERIALCONSOLE.print("8 - CSV Output : ");
        SERIALCONSOLE.println(CSVdebug);
        SERIALCONSOLE.print("9 - Decimal Places to Show : ");
        SERIALCONSOLE.println(debugdigits);
        SERIALCONSOLE.print("d - CSV Delimiter : ");
        if (delim == 1)
        {
          SERIALCONSOLE.println("Space");
        }
        else
        {
          SERIALCONSOLE.println("Comma");
        }
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 4;
        break;

      case 99: //c for calibrate zero offset
        while (Serial.available()) {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Current Sensor Calibration Menu");
        SERIALCONSOLE.println("c - To calibrate sensor offset");
        SERIALCONSOLE.print("s - Current Sensor Type : ");
        switch (settings.cursens)
        {
          case Analoguedual:
            SERIALCONSOLE.println(" Analogue Dual Current Sensor ");
            break;
          case Analoguesing:
            SERIALCONSOLE.println(" Analogue Single Current Sensor ");
            break;
          case Canbus:
            SERIALCONSOLE.println(" Canbus Current Sensor ");
            break;
          default:
            SERIALCONSOLE.println("Undefined");
            break;
        }
        SERIALCONSOLE.print("1 - invert current : ");
        SERIALCONSOLE.println(settings.invertcur);
        SERIALCONSOLE.print("2 - Pure Voltage based SOC : ");
        SERIALCONSOLE.println(settings.voltsoc);
        SERIALCONSOLE.print("3 - Current Multiplication : ");
        SERIALCONSOLE.println(settings.ncur);
        if (settings.cursens == Analoguesing || settings.cursens == Analoguedual)
        {
          SERIALCONSOLE.print("4 - Analogue Low Range Conv : ");
          SERIALCONSOLE.print(settings.convlow * 0.01, 2);
          SERIALCONSOLE.println(" mV / A");
        }
        if ( settings.cursens == Analoguedual)
        {
          SERIALCONSOLE.print("5 - Analogue High Range Conv : ");
          SERIALCONSOLE.print(settings.convhigh * 0.01, 2);
          SERIALCONSOLE.println(" mV / A");

        }
        if (settings.cursens == Analoguesing || settings.cursens == Analoguedual)
        {
          SERIALCONSOLE.print("6 - Current Sensor Deadband : ");
          SERIALCONSOLE.print(settings.CurDead);
          SERIALCONSOLE.println(" mV");

        }
        if ( settings.cursens == Analoguedual)
        {

          SERIALCONSOLE.print("8 - Current Channel ChangeOver : ");
          SERIALCONSOLE.print(settings.changecur * 0.001);
          SERIALCONSOLE.println(" A");
        }

        if ( settings.cursens == Canbus)
        {
          SERIALCONSOLE.print("7 - Can Current Sensor : ");
          if (settings.curcan == LemCAB300)
          {
            SERIALCONSOLE.println(" LEM CAB300 / 500 series ");
          }
          else  if (settings.curcan == LemCAB500)
          {
            SERIALCONSOLE.println(" LEM CAB500 Special ");
          }
          else if (settings.curcan == IsaScale)
          {
            SERIALCONSOLE.println(" IsaScale IVT - S ");
          }
          else if (settings.curcan == VictronLynx)
          {
            SERIALCONSOLE.println(" Victron Lynx VE.CAN Shunt");
          }
        }
        SERIALCONSOLE.println("q - Go back to menu");
        menuload = 2;
        break;


      case 98: //c for calibrate zero offset
        while (Serial.available())
        {
          Serial.read();
        }
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.println("Battery Settings Menu");
        SERIALCONSOLE.println("r - Reset AH counter");
        SERIALCONSOLE.println("f - Reset to Coded Settings");
        SERIALCONSOLE.println("q - Go back to menu");
        SERIALCONSOLE.println();
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("1 - Cell Over Voltage Setpoint : ");
        SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("2 - Cell Under Voltage Setpoint : ");
        SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("3 - Over Temperature Setpoint : ");
        SERIALCONSOLE.print(settings.OverTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("4 - Under Temperature Setpoint : ");
        SERIALCONSOLE.print(settings.UnderTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("5 - Cell Balance Voltage Setpoint : ");
        SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("6 - Balance STOP: down to difference with lowest cell : ");
        SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("l - Balance START: difference with lowest cell greater than : ");
        SERIALCONSOLE.print(settings.balanceStart * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("7 - Ah Capacity per String : ");
        SERIALCONSOLE.print(settings.CAP);
        SERIALCONSOLE.print("Ah");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("8 - Max Discharge Current PER STRING : ");
        SERIALCONSOLE.print(settings.discurrentmax * 0.1);
        SERIALCONSOLE.print("A");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("9 - Cell Discharge Voltage Limit Setpoint : ");
        SERIALCONSOLE.print(settings.DischVsetpoint * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("0 - Strings in parallel : ");
        SERIALCONSOLE.print(settings.Pstrings);
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("a - Cells in Series per String (6=24V, 12=48V) : ");
        SERIALCONSOLE.print(settings.Scells );
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("b - setpoint 1 : ");
        SERIALCONSOLE.print(settings.socvolt[0] );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("c - SOC setpoint 1 : ");
        SERIALCONSOLE.print(settings.socvolt[1] );
        SERIALCONSOLE.print(" % ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("d - setpoint 2 : ");
        SERIALCONSOLE.print(settings.socvolt[2] );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("e - SOC setpoint 2 : ");
        SERIALCONSOLE.print(settings.socvolt[3] );
        SERIALCONSOLE.print(" % ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("g - Storage Setpoint : ");
        SERIALCONSOLE.print(settings.StoreVsetpoint * 1000, 0 );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("h - Discharge Current Taper Offset : ");
        SERIALCONSOLE.print(settings.DisTaper * 1000, 0 );
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("j - Discharge Current Temperature Derate : ");
        SERIALCONSOLE.print(settings.DisTSetpoint);
        SERIALCONSOLE.print("C");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print("k - Cell Discharge Voltage Hysteresis : ");
        SERIALCONSOLE.print(settings.DischHys * 1000, 0);
        SERIALCONSOLE.print("mV");
        SERIALCONSOLE.println("  ");

        SERIALCONSOLE.println();
        menuload = 3;
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  // NEW: direct commands outside the menu (used by the Node-RED dashboard):
  //   'J' = JSON output on (and save), 'T' = back to text output (and save)
  //   'B<start>,<stop>' = balancing start / stop difference in mV (and save), e.g. "B7,3"
  //   'S<soc>,<current A>' = SOC (%) and battery current (A, + = charging) from outside, e.g. a Lynx shunt
  //                          on the GX via Node-RED. Used while fresh (< 60 s). Not saved.
  if (menuload == 0 && incomingByte == 'S')
  {
    float soc = Serial.parseFloat();
    float amps = Serial.parseFloat();
    if (soc >= 0.0f && soc <= 100.0f)
    {
      extSoc = soc;
      extSocMs = millis();
      // the current goes the same way as a Lynx on the CAN bus
      lastCurrentMs = millis();
      CANmilliamps = (signed long)(amps * 1000.0f);
      if (settings.cursens == Canbus)
      {
        RawCur = CANmilliamps;
        getcurrent();
      }
    }
  }
  //   'C<cells>' = cells in series per string (and save): C6 = 24 V, C12 = 48 V; strings are counted again
  if (menuload == 0 && incomingByte == 'C')
  {
    long c = Serial.parseInt();
    SERIALCONSOLE.println();
    if (c >= 6 && c <= 96 && (c % 6) == 0)
    {
      settings.Scells = (int)c;
      applyTopology();
      EEPROM.put(0, settings);
      eepromWasReset = false;
      SERIALCONSOLE.print("Cells in series: "); SERIALCONSOLE.print(c); SERIALCONSOLE.println(" (saved)");
    }
    else SERIALCONSOLE.println("Cells in series: invalid value (C6, C12, ... multiples of 6) - not changed");
    jsonTimer = 0;
  }
  //   'K<kbit>' = CAN speed (and save): K250 (VE.Can), K500 (CAN-bus BMS / LG RESU), also K125, K1000
  if (menuload == 0 && incomingByte == 'K')
  {
    long k = Serial.parseInt();
    SERIALCONSOLE.println();
    if (k == 125 || k == 250 || k == 500 || k == 1000)
    {
      settings.canSpeed = (uint32_t)k * 1000;
      bmsCan.end();
      bmsCan.begin(settings.canSpeed);
      EEPROM.put(0, settings);
      eepromWasReset = false;
      SERIALCONSOLE.print("CAN speed: "); SERIALCONSOLE.print(k); SERIALCONSOLE.println(" kbit/s (saved)");
    }
    else SERIALCONSOLE.println("CAN speed: invalid value (K125, K250, K500, K1000) - not changed");
    jsonTimer = 0;
  }
  //   'P<n>' = CAN protocol to the inverter (and save): P0 = Victron, P1 = LG RESU LV (SolarEdge), P1 sets 500 kbit/s
  if (menuload == 0 && incomingByte == 'P')
  {
    long p = Serial.parseInt();
    SERIALCONSOLE.println();
    if (p == 0 || p == 1)
    {
      settings.InverterProto = (uint8_t)p;
      canLogUsed = 0;
      lgKeepAliveMs = 0;
      eepromWasReset = false;
      if (p == 1 && settings.canSpeed != 500000)
      {
        // LG RESU LV always talks 500 kbit/s
        settings.canSpeed = 500000;
        bmsCan.end();
        bmsCan.begin(settings.canSpeed);
      }
      EEPROM.put(0, settings);
      SERIALCONSOLE.println(p ? "CAN protocol: LG RESU LV (SolarEdge), 500 kbit/s (saved)" : "CAN protocol: Victron (saved)");
    }
    else SERIALCONSOLE.println("CAN protocol: invalid value (P0 = Victron, P1 = LG RESU LV) - not changed");
    jsonTimer = 0;
  }
  if (menuload == 0 && incomingByte == 'B')
  {
    long bStart = Serial.parseInt();
    long bStop = Serial.parseInt();
    SERIALCONSOLE.println();
    if (bStop >= 2 && bStop <= 50 && bStart > bStop && bStart <= 100)
    {
      settings.balanceStart = bStart / 1000.0f;
      settings.balanceHyst = bStop / 1000.0f;
      EEPROM.put(0, settings);
      eepromWasReset = false;
      SERIALCONSOLE.print("Balancing: start > "); SERIALCONSOLE.print(bStart);
      SERIALCONSOLE.print(" mV, down to "); SERIALCONSOLE.print(bStop); SERIALCONSOLE.println(" mV (saved)");
    }
    else
    {
      SERIALCONSOLE.println("Balancing: invalid values (stop 2-50 mV, start > stop, max 100 mV) - not changed");
    }
    jsonTimer = 0; // send JSON immediately, so the dashboard shows the new values
  }
  if (menuload == 0 && (incomingByte == 'J' || incomingByte == 'T'))
  {
    settings.JsonOut = (incomingByte == 'J') ? 1 : 0;
    debug = (settings.JsonOut == 1) ? 0 : 1;
    EEPROM.put(0, settings);
    SERIALCONSOLE.println();
    SERIALCONSOLE.println(settings.JsonOut ? "JSON output on" : "JSON output off (text)");
    jsonTimer = 0; // send immediately
  }

  if (incomingByte == 115 && menuload == 0)
  {
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("MENU");
    SERIALCONSOLE.println("Debugging Paused");
    SERIALCONSOLE.print("Firmware Version : ");
    SERIALCONSOLE.print(firmver);
    SERIALCONSOLE.print("  compiled ");
    SERIALCONSOLE.print(__DATE__);
    SERIALCONSOLE.print(" ");
    SERIALCONSOLE.print(__TIME__);
    SERIALCONSOLE.print("  (VenusOS: v");
    SERIALCONSOLE.print(fwYear);
    SERIALCONSOLE.print(".");
    SERIALCONSOLE.print(fwWeek);
    SERIALCONSOLE.println(")");
    SERIALCONSOLE.println("b - Battery Settings");
    SERIALCONSOLE.println("a - Alarm and Warning Settings");
    SERIALCONSOLE.println("e - Charging Settings");
    SERIALCONSOLE.println("c - Current Sensor Calibration");
    SERIALCONSOLE.println("k - Contactor and Gauge Settings");
    SERIALCONSOLE.println("i - Ignore Value Settings");
    SERIALCONSOLE.println("d - Debug Settings");
    SERIALCONSOLE.println("m - Modules & Strings");
    SERIALCONSOLE.println("J / T - (outside the menu) JSON output on / back to text");
    SERIALCONSOLE.println("x - Experimental Settings");
    SERIALCONSOLE.println("R - Restart BMS");
    SERIALCONSOLE.println("q - exit menu");
    debug = 0;
    menuload = 1;
  }
}

int pgnFromCANId(int canId)
{
  if ((canId & 0x10000000) == 0x10000000)
  {
    return (canId & 0x03FFFF00) >> 8;
  }
  else
  {
    return canId; // not sure if this is really right?
  }
}

void canread()
{
  // inMsg has just been read by bmsCan.read() in loop()
  if (settings.InverterProto == 1)
  {
    canLogAdd(inMsg);                       // what does the inverter send? (JSON "cx")
    if (inMsg.id == 0x305 && !inMsg.ext) lgKeepAliveMs = millis();
  }
  // Read data: len = data length, buf = data byte(s)
  if ( settings.cursens == Canbus)
  {
    if (settings.curcan == 1)
    {
      switch (inMsg.id)
      {
        case 0x3c0:
          CAB300();
          break;

        case 0x3c1:
          CAB300();
          break;

        case 0x3c2:
          CAB300();
          break;

        default:
          break;
      }
    }
    if (settings.curcan == 2)
    {
      switch (inMsg.id)
      {
        case 0x3c0:
          CAB500();
          break;

        case 0x3c1:
          CAB500();
          break;

        case 0x3c2:
          CAB500();
          break;

        default:
          break;
      }
    }
    if (settings.curcan == 3)
    {
      switch (inMsg.id)
      {
        case 0x521: //
          CANmilliamps = (long)((inMsg.buf[2] << 24) | (inMsg.buf[3] << 16) | (inMsg.buf[4] << 8) | (inMsg.buf[5]));
          if ( settings.cursens == Canbus)
          {
            RawCur = CANmilliamps;
            getcurrent();
          }
          break;
        case 0x3C3: // Jaguar Ipace ISA shunt current reading
          CANmilliamps = inMsg.buf[5] + (inMsg.buf[4] << 8) + (inMsg.buf[3] << 16) + (inMsg.buf[2] << 24);
          if ( settings.cursens == Canbus)
          {
            RawCur = CANmilliamps;
            getcurrent();
          }
          break;

        case 0x522: //
          voltage1 = (long)((inMsg.buf[2] << 24) | (inMsg.buf[3] << 16) | (inMsg.buf[4] << 8) | (inMsg.buf[5]));
          break;
        case 0x523: //
          voltage2 = (long)((inMsg.buf[2] << 24) | (inMsg.buf[3] << 16) | (inMsg.buf[4] << 8) | (inMsg.buf[5]));
          break;
        default:
          break;
      }
    }
    if (settings.curcan == 4)
    {
      if (pgnFromCANId(inMsg.id) == 0x1F214 && inMsg.buf[0] == 0) // Check PGN and only use the first packet of each sequence
      {
        handleVictronLynx();
      }
    }
  }

  if (inMsg.id == 0x309)
  {
    Rx309();
  }


  if (debug == 1)
  {
    if (candebug == 1)
    {
      Serial.print(millis());
      if ((inMsg.id & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID : 0x % .8lX  DLC : % 1d  Data : ", (inMsg.id & 0x1FFFFFFF), inMsg.len);
      else
        sprintf(msgString, ", 0x % .3lX, false, % 1d", inMsg.id, inMsg.len);

      Serial.print(msgString);

      if ((inMsg.id & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      } else {
        for (byte i = 0; i < inMsg.len; i++) {
          sprintf(msgString, ", 0x % .2X", inMsg.buf[i]);
          Serial.print(msgString);
        }
      }

      Serial.println();
    }
  }
}

void Rx309()
{
  if (SOCset == 1)
  {
    if (inMsg.buf[0] & 0x01)
    {
      CanOnReq = true;
      CanOnRev = true;
      CanOntimeout = millis();
    }
    else
    {
      CanOnReq = false;
      CanOnRev = true;
      CanOntimeout = millis();
    }
  }
}

void CAB300()
{
  lastCurrentMs = millis();
  for (int i = 0; i < 4; i++)
  {
    inbox = (inbox << 8) | inMsg.buf[i];
  }
  CANmilliamps = inbox;
  if (CANmilliamps > 0x80000000)
  {
    CANmilliamps -= 0x80000000;
  }
  else
  {
    CANmilliamps = (0x80000000 - CANmilliamps) * -1;
  }
  if ( settings.cursens == Canbus)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA ");
  }
}

void CAB500()
{
  lastCurrentMs = millis();
  inbox = 0;
  for (int i = 1; i < 4; i++)
  {
    inbox = (inbox << 8) | inMsg.buf[i];
  }
  CANmilliamps = inbox;
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps, HEX);
  }
  if (CANmilliamps > 0x800000)
  {
    CANmilliamps -= 0x800000;
  }
  else
  {
    CANmilliamps = (0x800000 - CANmilliamps) * -1;
  }
  if ( settings.cursens == Canbus)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA ");
  }
}

void handleVictronLynx()
{
  if (inMsg.buf[4] == 0xff && inMsg.buf[3] == 0xff) return;
  lastCurrentMs = millis();
  int16_t current = (int)inMsg.buf[4] << 8; // in 0.1A increments
  current |= inMsg.buf[3];
  CANmilliamps = current * 100;
  if (settings.cursens == Canbus)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA ");
  }
}

/*
  NEW: at startup, show the most important settings, with a warning when a value is illogical.
  The settings live in the EEPROM of the Teensy and survive flashing.
*/
void printKeySettings()
{
  SERIALCONSOLE.println();
  if (eepromWasReset)
  {
    SERIALCONSOLE.println("*** EEPROM reset ONCE to the default settings (CAN speed kept) ***");
  }
  SERIALCONSOLE.println("---- Settings (EEPROM) ----");
  SERIALCONSOLE.print("Cells in series per string: "); SERIALCONSOLE.print(settings.Scells);
  SERIALCONSOLE.print("   strings: "); SERIALCONSOLE.print(settings.Pstrings);
  SERIALCONSOLE.print(settings.AutoTopology ? " (auto)" : " (fixed)");
  SERIALCONSOLE.print("   capacity per string: "); SERIALCONSOLE.print(settings.CAP); SERIALCONSOLE.println(" Ah");
  SERIALCONSOLE.print("Charge voltage/cell: "); SERIALCONSOLE.print(settings.ChargeVsetpoint * 1000, 0);
  SERIALCONSOLE.print(" mV  (CVL max "); SERIALCONSOLE.print(settings.ChargeVsetpoint * settings.Scells, 1);
  SERIALCONSOLE.print(" V)   overvoltage: "); SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
  SERIALCONSOLE.print(" mV   undervoltage: "); SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
  SERIALCONSOLE.print(" mV   discharge limit: "); SERIALCONSOLE.print(settings.DischVsetpoint * 1000, 0); SERIALCONSOLE.println(" mV");
  SERIALCONSOLE.print("CCL/DCL per string: "); SERIALCONSOLE.print(settings.chargecurrentmax * 0.1, 1);
  SERIALCONSOLE.print(" / "); SERIALCONSOLE.print(settings.discurrentmax * 0.1, 1); SERIALCONSOLE.println(" A");
  SERIALCONSOLE.print("Balancing: from "); SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
  SERIALCONSOLE.print(" mV, start > "); SERIALCONSOLE.print(settings.balanceStart * 1000, 0);
  SERIALCONSOLE.print(" mV, down to "); SERIALCONSOLE.print(settings.balanceHyst * 1000, 0); SERIALCONSOLE.println(" mV");
  SERIALCONSOLE.print("ESS mode: "); SERIALCONSOLE.print(settings.ESSmode ? "on" : "off");
  SERIALCONSOLE.print("   charger type: "); SERIALCONSOLE.print(settings.chargertype);
  SERIALCONSOLE.print("   current sensor: "); SERIALCONSOLE.print(settings.cursens);
  SERIALCONSOLE.print("/"); SERIALCONSOLE.print(settings.curcan);
  SERIALCONSOLE.print("   CAN: "); SERIALCONSOLE.print(settings.canSpeed / 1000); SERIALCONSOLE.println(" kbit/s");

  // checks
  if (settings.ChargeVsetpoint > settings.OverVSetpoint - 0.05f)
    SERIALCONSOLE.println("!!! WARNING: charge voltage per cell is (almost) above overvoltage - CVL is capped at OverV - 50 mV");
  if (settings.ChargeVsetpoint < 3.5f || settings.ChargeVsetpoint > 4.2f)
    SERIALCONSOLE.println("!!! WARNING: charge voltage per cell outside 3.50-4.20 V - check menu e -> 1");
  if (settings.Scells % 6 != 0)
    SERIALCONSOLE.println("!!! WARNING: cells in series is not a multiple of 6 - check menu b -> a");
  if (settings.DischVsetpoint < settings.UnderVSetpoint)
    SERIALCONSOLE.println("!!! WARNING: discharge limit lower than undervoltage");
  SERIALCONSOLE.println("---------------------------");
}

/*
  NEW: Victron-style CVL control (settings.CVLControl = 1).
  - Normal: CVL = ChargeVsetpoint x Scells (storage mode: StoreVsetpoint x Scells).
  - As soon as the highest cell is above ChargeVsetpoint, the CVL is first put at most
    CVL_START_MARGIN above the battery voltage, and then drops by at most CVL_DOWN_RATE
    (0.1 V/s) to (battery voltage - excess of that cell), so that this cell goes
    back to the target. The Victron chargers then neatly hold the
    voltage; the CCL stays open.
  - Is the highest cell CVL_BAND below the target again, the CVL rises slowly (CVL_UP_RATE);
    is it more than CVL_FAR_BAND (50 mV) below the target, the CVL goes straight to the maximum.
  - Upper limit: ChargeVsetpoint x Scells. Lower limit only as a safety net: UnderVSetpoint x Scells.
  - CCL = 0 only at real overvoltage (OverVSetpoint, see currentlimit()).
*/
void cvlControl()
{
  float cellTarget = (storagemode == 1) ? settings.StoreVsetpoint : settings.ChargeVsetpoint;
  // Safety: the target may never be above OverVSetpoint - 50 mV, whatever is set
  if (cellTarget > settings.OverVSetpoint - 0.05f) cellTarget = settings.OverVSetpoint - 0.05f;
  float cvlMax = cellTarget * settings.Scells;
  // Lower limit is ONLY a safety net against a wrong measurement. With a badly balanced
  // pack (e.g. 5 cells at 3.5 V, 1 at 4.15 V) the CVL must be able to go well below
  // (target - hysteresis) x Scells, otherwise charging continues.
  float cvlMin = settings.UnderVSetpoint * settings.Scells;

  unsigned long now = millis();
  float dt = (cvlTimer == 0) ? 0.0f : (now - cvlTimer) / 1000.0f;
  cvlTimer = now;
  if (dt > 2.0f) dt = 2.0f;

  if (settings.CVLControl == 0 || cvlActual <= 0.0f || bms.getStringsOnline() == 0)
  {
    cvlActual = cvlMax; // fixed CVL (original behaviour) / start value
  }
  else
  {
    float hc = bms.getHighCellVolt();
    if (hc > cellTarget)
    {
      // goal: battery voltage minus the excess of the highest cell, so that this cell
      // goes back to the target. Never lower further than that (no needless collapse).
      float cvlGoal = bms.getPackVoltage() - (hc - cellTarget);
      // A CVL far above the battery voltage limits nothing: first move to just
      // above the battery voltage, then drop by 0.1 V/s from there. Otherwise the
      // chargers keep supplying full current during the ramp (dangerous for the high cell).
      float cvlStart = bms.getPackVoltage() + CVL_START_MARGIN;
      if (cvlActual > cvlStart) cvlActual = cvlStart;
      if (cvlActual > cvlGoal)
      {
        cvlActual -= CVL_DOWN_RATE * dt;
        if (cvlActual < cvlGoal) cvlActual = cvlGoal;
      }
    }
    else if (hc < cellTarget - CVL_FAR_BAND)
    {
      // nowhere near full: no reason to hold back, straight to the maximum
      // (e.g. after a switch from 24 V to 48 V, or after the battery was discharged)
      cvlActual = cvlMax;
    }
    else if (hc < cellTarget - CVL_BAND)
    {
      // close to full: raise slowly again (prevents hunting)
      cvlActual += CVL_UP_RATE * dt;
    }
  }
  if (cvlActual > cvlMax) cvlActual = cvlMax;
  if (cvlActual < cvlMin) cvlActual = cvlMin;
}

void currentlimit()
{
  if (bmsstatus == Error)
  {
    discurrent = 0;
    chargecurrent = 0;
  }
  /*
    settings.PulseCh = 600; //Peak Charge current in 0.1A
    settings.PulseChDur = 5000; //Ms of discharge pulse derating
    settings.PulseDi = 600; //Peak Charge current in 0.1A
    settings.PulseDiDur = 5000; //Ms of discharge pulse derating
  */
  else
  {

    ///Start at no derating///
    discurrent = settings.discurrentmax;

    if (chargecurrentlimit == false)
    {
      chargecurrent = settings.chargecurrentmax;
    }
    else
    {
      chargecurrent = settings.chargecurrent2max;
    }

    ///////All hard limits to into zeros
    if (bms.getLowTemperature() < settings.UnderTSetpoint)
    {
      //discurrent = 0; Request Daniel
      chargecurrent = 0;
    }
    if (bms.getHighTemperature() > settings.OverTSetpoint)
    {
      discurrent = 0;
      chargecurrent = 0;
    }
    // NEW: real overvoltage (default 4.20 V per cell) -> CCL = 0, the signal for Victron
    // to switch off all chargers. Released only when the highest cell is OV_RELEASE lower again.
    if (bms.getHighCellVolt() >= settings.OverVSetpoint)
    {
      overVoltLatch = true;
    }
    else if (bms.getHighCellVolt() < settings.OverVSetpoint - OV_RELEASE)
    {
      overVoltLatch = false;
    }
    if (overVoltLatch)
    {
      chargecurrent = 0;
    }
    if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < dischargeCellV())
    {
      discurrent = 0;
    }


    //Modifying discharge current///

    if (discurrent > 0)
    {
      //Temperature based///

      if (bms.getHighTemperature() > settings.DisTSetpoint)
      {
        discurrent = discurrent - map(bms.getHighTemperature(), settings.DisTSetpoint, settings.OverTSetpoint, 0, settings.discurrentmax);
      }
      //Voltagee based///
      if (bms.getLowCellVolt() < (dischargeCellV() + settings.DisTaper))
      {
        discurrent = discurrent - map(bms.getLowCellVolt(), dischargeCellV(), (dischargeCellV() + settings.DisTaper), settings.discurrentmax, 0);
      }
    }

    //Modifying Charge current///

    if (chargecurrent > 0)
    {
      if (chargecurrentlimit == false)
      {
        //Temperature based///
        if (bms.getLowTemperature() < settings.ChargeTSetpoint)
        {
          chargecurrent = chargecurrent - map(bms.getLowTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, settings.chargecurrentmax, 0);
        }
        //Voltagee based/// (only without CVL control; with CVL control the CVL takes care of it)
        if (settings.CVLControl == 1)
        {
          // no CCL taper: the CVL is lowered in cvlControl()
        }
        else if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, settings.chargecurrentmax);
          }
        }
        else
        {
          if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, 0, (settings.chargecurrentmax - settings.chargecurrentend));
          }
        }
      }
      else
      {
        //Temperature based///
        if (bms.getLowTemperature() < settings.ChargeTSetpoint)
        {
          chargecurrent = chargecurrent - map(bms.getLowTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, settings.chargecurrent2max, 0);
        }
        //Voltagee based/// (only without CVL control; with CVL control the CVL takes care of it)
        if (settings.CVLControl == 1)
        {
          // no CCL taper: the CVL is lowered in cvlControl()
        }
        else if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, settings.chargecurrent2max);
          }
        }
        else
        {
          if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, 0, (settings.chargecurrent2max - settings.chargecurrentend));
          }
        }
      }
    }

  }
  // NEW: the calculation above is PER STRING. Clamp to 0..max per string
  // (the voltage taper can extrapolate above ChargeVsetpoint; previously that could
  // wrap around to a huge CCL with the old uint16 type).
  if (discurrent < 0) discurrent = 0;
  if (chargecurrent < 0) chargecurrent = 0;
  if (discurrent > settings.discurrentmax) discurrent = settings.discurrentmax;
  if (chargecurrent > settings.chargecurrentmax) chargecurrent = settings.chargecurrentmax;
  // Very empty (lowest cell below UnderVSetpoint): charge slowly, but NEVER CCL = 0 for a low
  // battery. CCL = 0 only for a cell that is too high (or temperature).
  if (bms.getLowCellVolt() < settings.UnderVSetpoint && chargecurrent > settings.chargecurrentend)
  {
    chargecurrent = settings.chargecurrentend;
  }

  // Total for the battery = per string x number of strings.
  // ScaleLimits = 1: only count strings that are online; 0: always the set number.
  {
    int32_t nStrings = (settings.ScaleLimits == 1) ? bms.getStringsOnline() : bms.getStringsExpected();
    if (bms.getStringsOnline() == 0) nStrings = 0; // no information = no current
    discurrent = discurrent * nStrings;
    chargecurrent = chargecurrent * nStrings;
    if (discurrent > 32000) discurrent = 32000;      // 0x351 field: max 3200 A
    if (chargecurrent > 32000) chargecurrent = 32000;
  }

  ///No negative currents///

  if (discurrent < 0)
  {
    discurrent = 0;
  }
  if (chargecurrent < 0)
  {
    chargecurrent = 0;
  }

  //Charge current derate for Control Pilot AC limit

  if (accurlim > 0)
  {
    chargerpower = accurlim * settings.chargerACv * settings.chargereff * 0.01;
    tempchargecurrent = (chargerpower * 10) / (bms.getAvgCellVolt() * settings.Scells);

    if ( chargecurrent > tempchargecurrent)
    {
      chargecurrent = tempchargecurrent;
    }
  }


}



void inputdebug()
{
  Serial.println();
  Serial.print("Input : ");
  if (digitalRead(IN1))
  {
    Serial.print("1 ON  ");
  }
  else
  {
    Serial.print("1 OFF ");
  }
  if (digitalRead(IN3))
  {
    Serial.print("2 ON  ");
  }
  else
  {
    Serial.print("2 OFF ");
  }
  if (digitalRead(IN3))
  {
    Serial.print("3 ON  ");
  }
  else
  {
    Serial.print("3 OFF ");
  }
  if (digitalRead(IN4))
  {
    Serial.print("4 ON  ");
  }
  else
  {
    Serial.print("4 OFF ");
  }
  Serial.println();
}

void outputdebug()
{
  if (outputstate < 5)
  {
    halOutWrite(OUT1, HIGH);
    halOutWrite(OUT2, HIGH);
    halOutWrite(OUT3, HIGH);
    halOutWrite(OUT4, HIGH);
    analogWrite(OUT5, 255);
    analogWrite(OUT6, 255);
    analogWrite(OUT7, 255);
    analogWrite(OUT8, 255);
    outputstate ++;
  }
  else
  {
    halOutWrite(OUT1, LOW);
    halOutWrite(OUT2, LOW);
    halOutWrite(OUT3, LOW);
    halOutWrite(OUT4, LOW);
    analogWrite(OUT5, 0);
    analogWrite(OUT6, 0);
    analogWrite(OUT7, 0);
    analogWrite(OUT8, 0);
    outputstate ++;
  }
  if (outputstate > 10)
  {
    outputstate = 0;
  }
}

void resetwdog()
{
  halWatchdogKick();
}

void pwmcomms()
{
  int p = 0;
  p = map((currentact * 0.001), pwmcurmin, pwmcurmax, 50 , 255);
  analogWrite(OUT7, p);
  /*
    Serial.println();
    Serial.print(p*100/255);
    Serial.print(" OUT8 ");
  */

  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    analogWrite(OUT7, 255); //12V to 10V converter 1.5V
  }
  else
  {
    p = map(SOC, 0, 100, 220, 50);
    analogWrite(OUT8, p); //2V to 10V converter 1.5-10V
  }
  /*
    Serial.println();
    Serial.print(p);
    Serial.print(" OUT7 ");
  */
}

void dashupdate()
{
  SERIALDISP.write("stat.txt=");
  SERIALDISP.write(0x22);
  if (settings.ESSmode == 1)
  {
    switch (bmsstatus)
    {
      case (Boot):
        SERIALDISP.print(" Active ");
        break;
      case (Error):
        SERIALDISP.print(" Error ");
        break;
    }
  }
  else
  {
    switch (bmsstatus)
    {
      case (Boot):
        SERIALDISP.print(" Boot ");
        break;

      case (Ready):
        SERIALDISP.print(" Ready ");
        break;

      case (Precharge):
        SERIALDISP.print(" Precharge ");
        break;

      case (Drive):
        SERIALDISP.print(" Drive ");
        break;

      case (Charge):
        SERIALDISP.print(" Charge ");
        break;

      case (Error):
        SERIALDISP.print(" Error ");
        break;
    }
  }
  SERIALDISP.write(0x22);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("soc.val=");
  SERIALDISP.print(SOC);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("soc1.val=");
  SERIALDISP.print(SOC);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("current.val=");
  SERIALDISP.print(currentact / 100, 0);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("temp.val=");
  SERIALDISP.print(bms.getAvgTemperature(), 0);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("templow.val=");
  SERIALDISP.print(bms.getLowTemperature(), 0);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("temphigh.val=");
  SERIALDISP.print(bms.getHighTemperature(), 0);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("volt.val=");
  SERIALDISP.print(bms.getPackVoltage() * 10, 0);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("lowcell.val=");
  SERIALDISP.print(bms.getLowCellVolt() * 1000, 0);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("highcell.val=");
  SERIALDISP.print(bms.getHighCellVolt() * 1000, 0);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("firm.val=");
  SERIALDISP.print(firmver);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("celldelta.val=");
  SERIALDISP.print((bms.getHighCellVolt() - bms.getLowCellVolt()) * 1000, 0);
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
  SERIALDISP.print("cellbal.val=");
  SERIALDISP.print(bms.getBalancing());
  SERIALDISP.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  SERIALDISP.write(0xff);
  SERIALDISP.write(0xff);
}


void balancing()
{
  if (balancecells == 1)
  {
    if (debug == 1)
    {
      bms.balanceCells(settings.balanceDuty, 0);
    }
    else
    {
      bms.balanceCells(settings.balanceDuty, 0);
    }
  }
  else
  {
    bms.StopBalancing();
  }
}


void chargercomms()
{

  if (settings.chargertype == Elcon)
  {
    msg.id  =  0x1806E5F4; //broadcast to all Elteks
    msg.len = 8;
    msg.ext = 1;
    msg.buf[0] = highByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[1] = lowByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[2] = highByte(chargecurrent / ncharger);
    msg.buf[3] = lowByte(chargecurrent / ncharger);
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    bmsCan.write(msg);
    msg.ext = 0;
  }

  if (settings.chargertype == Eltek)
  {
    msg.id  = 0x2FF; //broadcast to all Elteks
    msg.len = 7;
    msg.buf[0] = 0x01;
    msg.buf[1] = lowByte(1000);
    msg.buf[2] = highByte(1000);
    msg.buf[3] = lowByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[4] = highByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[5] = lowByte(chargecurrent / ncharger);
    msg.buf[6] = highByte(chargecurrent / ncharger);

    bmsCan.write(msg);
  }
  if (settings.chargertype == BrusaNLG5)
  {
    msg.id  = chargerid1;
    msg.len = 7;
    msg.buf[0] = 0x80;
    /*
      if (chargertoggle == 0)
      {
      msg.buf[0] = 0x80;
      chargertoggle++;
      }
      else
      {
      msg.buf[0] = 0xC0;
      chargertoggle = 0;
      }
    */
    if (digitalRead(IN2) == LOW)//Gen OFF
    {
      msg.buf[1] = highByte(maxac1 * 10);
      msg.buf[2] = lowByte(maxac1 * 10);
    }
    else
    {
      msg.buf[1] = highByte(maxac2 * 10);
      msg.buf[2] = lowByte(maxac2 * 10);
    }
    msg.buf[5] = highByte(chargecurrent / ncharger);
    msg.buf[6] = lowByte(chargecurrent / ncharger);
    msg.buf[3] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells ) - chargerendbulk) * 10));
    msg.buf[4] = lowByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells ) - chargerendbulk)  * 10));
    bmsCan.write(msg);

    delay(2);

    msg.id  = chargerid2;
    msg.len = 7;
    msg.buf[0] = 0x80;
    if (digitalRead(IN2) == LOW)//Gen OFF
    {
      msg.buf[1] = highByte(maxac1 * 10);
      msg.buf[2] = lowByte(maxac1 * 10);
    }
    else
    {
      msg.buf[1] = highByte(maxac2 * 10);
      msg.buf[2] = lowByte(maxac2 * 10);
    }
    msg.buf[3] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells ) - chargerend) * 10));
    msg.buf[4] = lowByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells ) - chargerend) * 10));
    msg.buf[5] = highByte(chargecurrent / ncharger);
    msg.buf[6] = lowByte(chargecurrent / ncharger);
    bmsCan.write(msg);
  }
  if (settings.chargertype == ChevyVolt)
  {
    msg.id  = 0x30E;
    msg.len = 1;
    msg.buf[0] = 0x02; //only HV charging , 0x03 hv and 12V charging
    bmsCan.write(msg);

    msg.id  = 0x304;
    msg.len = 4;
    msg.buf[0] = 0x40; //fixed
    if ((chargecurrent * 2) > 255)
    {
      msg.buf[1] = 255;
    }
    else
    {
      msg.buf[1] = (chargecurrent * 2);
    }
    if ((settings.ChargeVsetpoint * settings.Scells ) > 200)
    {
      msg.buf[2] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 2));
      msg.buf[3] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 2));
    }
    else
    {
      msg.buf[2] = highByte( 400);
      msg.buf[3] = lowByte( 400);
    }
    bmsCan.write(msg);
  }

  if (settings.chargertype == Coda)
  {
    msg.id  = 0x050;
    msg.len = 8;
    msg.buf[0] = 0x00;
    msg.buf[1] = 0xDC;
    if ((settings.ChargeVsetpoint * settings.Scells ) > 200)
    {
      msg.buf[2] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
      msg.buf[3] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
    }
    else
    {
      msg.buf[2] = highByte( 400);
      msg.buf[3] = lowByte( 400);
    }
    msg.buf[4] = 0x00;
    if ((settings.ChargeVsetpoint * settings.Scells)*chargecurrent < 3300)
    {
      msg.buf[5] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) * chargecurrent) / 240));
      msg.buf[6] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) * chargecurrent) / 240));
    }
    else //15 A AC limit
    {
      msg.buf[5] = 0x00;
      msg.buf[6] = 0x96;
    }
    msg.buf[7] = 0x01; //HV charging
    bmsCan.write(msg);
  }
}


void isrCP ()
{
  if (  digitalRead(IN4) == LOW)
  {
    duration = micros() - pilottimer;
    pilottimer = micros();
  }
  else
  {
    accurlim = ((duration - (micros() - pilottimer + 35)) * 60) / duration; //pilottimer + "xx" optocoupler decade ms
  }
}  // ******** end of isr CP ********

void low_voltage_isr(void) {
  EEPROM.update(1000, uint8_t(SOC));   // the interrupt flags are cleared in hal.cpp
}
