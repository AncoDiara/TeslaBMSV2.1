#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"

/*
  TeslaBMS_Victron - BMSModuleManager

  Changes compared to TeslaBMSV2:
  - Topology: modules are grouped into STRINGS (setTopology).
      modules per string = Scells / 6  (6 = 24 V, 12 = 48 V)
      number of strings  = Pstrings
      Address 1..M = string 1, M+1..2M = string 2, etc. (M = modules per string)
      -> the order of the communication chain determines the grouping.
  - Detection: CRC check on every reply, receive buffer is flushed,
    renumbering no longer happens automatically at every start (only at startup
    when NO module is found at all, or manually via the menu).
  - Online tracking: a module is offline after MODULE_MAX_FAILS failed reads
    in a row. A string is online only if ALL of its modules are online.
  - All calculations (cell min/max, temperature, pack voltage) use only
    online modules of the configured strings.
  - Pack voltage = average of the complete strings (not sum / Pstrings).
  - Balancing per string, relative to the lowest cell in that same string.
  - rescan(): periodically looks for missing modules again (also modules that
    lost their address after a power interruption).
*/

extern EEPROMSettings settings;
extern void resetwdog(); // from the .ino: keep the watchdog alive during long scans

BMSModuleManager::BMSModuleManager()
{
  for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
    modules[i].setExists(false);
    modules[i].setAddress(i);
  }
  lowestPackVolt = 1000.0f;
  highestPackVolt = 0.0f;
  lowestPackTemp = 200.0f;
  highestPackTemp = -100.0f;
  isFaulted = false;
  Pstring = 1;
  numFoundModules = 0;
  modulesPerString = 1;
  expectedStrings = 1;
  stringsOnline = 0;
  modulesOnline = 0;
  extraModules = 0;
  packVolt = 0.0f;
  LowCellVolt = 0.0f;
  HighCellVolt = 0.0f;
  lowTemp = 0.0f;
  highTemp = 0.0f;
  CellsBalancing = 0;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++) balMask[x] = 0;
}

/////////////////////////////// Topology ///////////////////////////////

void BMSModuleManager::setTopology(int scellsPerString, int pstrings)
{
  modulesPerString = scellsPerString / 6;
  if (modulesPerString < 1) modulesPerString = 1;
  if (scellsPerString % 6 != 0)
  {
    SERIALCONSOLE.println("WARNING: Scells is not a multiple of 6 (Tesla module = 6 cells)");
  }
  expectedStrings = (pstrings < 1) ? 1 : pstrings;
  while (expectedStrings * modulesPerString > MAX_MODULE_ADDR) expectedStrings--;
  Pstring = expectedStrings;
}

int BMSModuleManager::getModulesPerString() { return modulesPerString; }
int BMSModuleManager::getStringsExpected()  { return expectedStrings; }
int BMSModuleManager::getExpectedModules()  { return modulesPerString * expectedStrings; }
int BMSModuleManager::getStringsOnline()    { return stringsOnline; }
int BMSModuleManager::getStringsOffline()   { return expectedStrings - stringsOnline; }
int BMSModuleManager::getModulesOnline()    { return modulesOnline; }
int BMSModuleManager::getExtraModules()     { return extraModules; }
bool BMSModuleManager::isModuleOnline(int addr) { return (addr >= 1 && addr <= MAX_MODULE_ADDR) ? modules[addr].isOnline() : false; }

bool BMSModuleManager::isConfiguredModule(int addr)
{
  return addr >= 1 && addr <= getExpectedModules();
}

int BMSModuleManager::stringOfModule(int addr)
{
  return (addr - 1) / modulesPerString; // 0-based string index
}

bool BMSModuleManager::isStringOnline(int s)
{
  if (s < 0 || s >= expectedStrings) return false;
  for (int m = 0; m < modulesPerString; m++)
  {
    if (!modules[s * modulesPerString + m + 1].isOnline()) return false;
  }
  return true;
}

void BMSModuleManager::updateStringStatus()
{
  modulesOnline = 0;
  extraModules = 0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (!modules[x].isExisting()) continue;
    if (isConfiguredModule(x))
    {
      if (modules[x].isOnline()) modulesOnline++;
    }
    else
    {
      extraModules++;
    }
  }
  stringsOnline = 0;
  for (int s = 0; s < expectedStrings; s++)
  {
    if (isStringOnline(s)) stringsOnline++;
  }
}

/////////////////////////////// Detection ///////////////////////////////

// Read register 0 of one address, with CRC check. attempts = number of tries.
bool BMSModuleManager::probeAddress(int addr, int attempts)
{
  uint8_t payload[3];
  uint8_t buff[8];
  for (int a = 0; a < attempts; a++)
  {
    payload[0] = addr << 1;
    payload[1] = 0;   //read registers starting at 0
    payload[2] = 1;   //read one byte
    BMSUtil::sendData(payload, 3, false);
    delay(2); // minimum wait as in the original
    int len = BMSUtil::getReplyTimed(buff, 5, 20);
    if (BMSUtil::checkReadReply(buff, len, addr, 0, 1)) return true;
    delay(2);
  }
  return false;
}

/*
   Iterate through all 62 possible board addresses (1-62) to see if they respond.
   Configured addresses get 3 attempts, the rest 1 (faster).
*/
void BMSModuleManager::findBoards()
{
  numFoundModules = 0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    bool found = probeAddress(x, isConfiguredModule(x) ? 3 : 1);
    modules[x].setExists(found);
    if (found) numFoundModules++;
    if ((x & 7) == 0) resetwdog();
  }
  updateStringStatus();
}

/*
   Give modules that are still at address 0 (new, or after a power interruption)
   the first free address. Fixed compared to the original:
   - success is verified by actually reading the new address
   - no infinite loop: at most MAX_MODULE_ADDR rounds and 3 failures
*/
void BMSModuleManager::setupBoards()
{
  uint8_t payload[3];
  uint8_t buff[10];
  int retLen;
  int failures = 0;

  for (int round = 0; round < MAX_MODULE_ADDR && failures < 3; round++)
  {
    resetwdog();
    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 1;
    retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
    if (!(retLen == 4 && buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1))
    {
      break; //nobody answers on address 0: all modules have an address
    }
    Logger::debug("00 found");

    // Choose the address: first the "holes" within the topology (a module that lost
    // its address after a power interruption gets its old place = the right string back),
    // only after that addresses outside the topology.
    int y = 0;
    for (int a = 1; a <= getExpectedModules(); a++)
    {
      if (modules[a].isOnline()) continue;
      if (probeAddress(a, 1)) // still responds at that address after all: mark as online
      {
        if (!modules[a].isExisting()) numFoundModules++;
        modules[a].setExists(true);
        continue;
      }
      y = a;
      break;
    }
    if (y == 0)
    {
      for (int a = getExpectedModules() + 1; a <= MAX_MODULE_ADDR; a++)
      {
        if (!modules[a].isExisting()) { y = a; break; }
      }
    }
    if (y == 0) break; // no free addresses left

    payload[0] = 0;
    payload[1] = REG_ADDR_CTRL;
    payload[2] = y | 0x80;
    BMSUtil::sendData(payload, 3, true);
    delay(3); // original timing (setupBoards)
    BMSUtil::getReplyTimed(buff, 4, 10);

    if (probeAddress(y, 3))
    {
      if (!modules[y].isExisting()) numFoundModules++;
      modules[y].setExists(true);
      Logger::debug("Address assigned");
      failures = 0;
    }
    else
    {
      failures++;
    }
  }
}

/*
   Force all modules to reset back to address 0 then set them all up in order so that the first module
   in line from the master board is 1, the second one 2, and so on.
   ONLY run this deliberately (menu), because it determines which module belongs to which string.
*/
void BMSModuleManager::renumberBoardIDs()
{
  uint8_t payload[3];
  uint8_t buff[8];

  for (int y = 1; y <= MAX_MODULE_ADDR; y++)
  {
    modules[y].setExists(false);
  }
  numFoundModules = 0;

  for (int attempts = 0; attempts < 3; attempts++)
  {
    memset(buff, 0, sizeof(buff));
    payload[0] = 0x3F << 1; //broadcast the reset command
    payload[1] = 0x3C;//reset
    payload[2] = 0xA5;//data to cause a reset
    BMSUtil::sendData(payload, 3, true);
    BMSUtil::getReplyTimed(buff, 4, 100);
    if (buff[0] == 0x7F && buff[1] == 0x3C && buff[2] == 0xA5 && buff[3] == 0x57) break;
    resetwdog();
  }
  delay(100); // let the modules start up after the reset
  resetwdog();

  setupBoards();
  updateStringStatus();
}

/*
  NEW: detection at startup.
  - Normally: just look up existing addresses (no renumbering).
  - Nothing found: first use -> renumber once.
  - Too few found: assign an address to any modules still at address 0.
*/
void BMSModuleManager::startupDetect()
{
  findBoards();
  if (numFoundModules == 0)
  {
    SERIALCONSOLE.println("No addressed modules found -> renumbering");
    renumberBoardIDs();
  }
  else if (modulesOnline < getExpectedModules())
  {
    setupBoards();
    updateStringStatus();
  }
  printTopology();
}

/*
  NEW: called periodically from the loop as long as not everything is complete.
  Looks for modules that are missing again, without disturbing the existing addresses.
*/
void BMSModuleManager::rescan()
{
  setupBoards(); // modules that fell back to address 0
  for (int x = 1; x <= getExpectedModules(); x++)
  {
    if (!modules[x].isOnline())
    {
      if (probeAddress(x, 2))
      {
        if (!modules[x].isExisting()) numFoundModules++;
        modules[x].setExists(true);
        modules[x].setIgnoreCell(settings.IgnoreVolt);
        modules[x].settempsensor(settings.IgnoreTemp);
      }
    }
    resetwdog();
  }
  updateStringStatus();
}

void BMSModuleManager::printTopology()
{
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Topology: ");
  SERIALCONSOLE.print(modulesPerString);
  SERIALCONSOLE.print(" module(s) per string x ");
  SERIALCONSOLE.print(expectedStrings);
  SERIALCONSOLE.print(" string(s) = ");
  SERIALCONSOLE.print(getExpectedModules());
  SERIALCONSOLE.println(" modules expected");
  for (int s = 0; s < expectedStrings; s++)
  {
    SERIALCONSOLE.print("  String ");
    SERIALCONSOLE.print(s + 1);
    SERIALCONSOLE.print(": ");
    for (int m = 0; m < modulesPerString; m++)
    {
      int a = s * modulesPerString + m + 1;
      SERIALCONSOLE.print("#");
      SERIALCONSOLE.print(a);
      SERIALCONSOLE.print(modules[a].isOnline() ? " OK  " : " MISSING  ");
    }
    SERIALCONSOLE.println(isStringOnline(s) ? " -> online" : " -> OFFLINE");
  }
  if (extraModules > 0)
  {
    SERIALCONSOLE.print("  WARNING: ");
    SERIALCONSOLE.print(extraModules);
    SERIALCONSOLE.println(" module(s) found outside the set topology (check Scells/Pstrings)");
  }
  SERIALCONSOLE.println();
}

/////////////////////////////// Housekeeping ///////////////////////////////

void BMSModuleManager::clearmodules()
{
  // only clear offline modules, so no stale values linger
  for (int y = 1; y <= MAX_MODULE_ADDR; y++)
  {
    if (modules[y].isExisting() && !modules[y].isOnline())
    {
      modules[y].clearmodule();
    }
  }
}

int BMSModuleManager::seriescells()
{
  spack = 0;
  for (int y = 1; y <= getExpectedModules(); y++)
  {
    if (modules[y].isOnline())
    {
      spack = spack + modules[y].getscells();
    }
  }
  return spack;
}

/*
  Balancing per string (passive: a resistor in the module bleeds the energy of
  the high cell as heat; no energy goes to the low cell).

  Rule per cell (compared with the lowest cell in ITS OWN string):
  - START bleeding when: cell > balanceVoltage  AND  cell > stringLow + balanceStart
  - STOP when:           cell <= stringLow + balanceHyst  (or cell <= balanceVoltage)
  Default: above 3.90 V, start at > 15 mV difference, balance down to 5 mV.
  The margin between start and stop prevents switching on/off due to measurement noise.
  Previously EVERY cell above the lowest cell (even 1 mV) was bled, so with
  6 modules almost always 5 of 6 cells per module were balancing.

  Parallel strings cannot balance each other. Incomplete strings are not balanced.
  CellsBalancing = number of CELLS (previously the sum of bitmasks).
*/
void BMSModuleManager::balanceCells(int duration, int debug)
{
  uint8_t payload[4];
  uint8_t buff[30];
  CellsBalancing = 0;
  float stopDelta = settings.balanceHyst;          // balance down to this difference
  if (stopDelta < 0.002f) stopDelta = 0.002f;       // below ~2 mV is measurement noise
  float startDelta = settings.balanceStart;         // start bleeding above this difference
  if (startDelta < stopDelta) startDelta = stopDelta;

  for (int s = 0; s < expectedStrings; s++)
  {
    if (!isStringOnline(s))
    {
      for (int m = 0; m < modulesPerString; m++) balMask[s * modulesPerString + m + 1] = 0;
      continue;
    }

    float stringLow = 10.0f;
    for (int m = 0; m < modulesPerString; m++)
    {
      float v = modules[s * modulesPerString + m + 1].getLowCellV();
      if (v < stringLow) stringLow = v;
    }

    for (int m = 0; m < modulesPerString; m++)
    {
      int y = s * modulesPerString + m + 1;
      uint8_t balance = 0; //bit 0 - 5 are to activate cell balancing 1-6
      for (int i = 0; i < 6; i++)
      {
        float v = modules[y].getCellVoltage(i);
        bool wasOn = balMask[y] & (1 << i);
        bool on;
        if (wasOn) on = (v > settings.balanceVoltage) && (v > stringLow + stopDelta);
        else       on = (v > settings.balanceVoltage) && (v > stringLow + startDelta);
        if (on) balance |= (1 << i);
      }
      balMask[y] = balance;

      if (debug == 1)
      {
        Serial.print(y);
        Serial.print(" - ");
        Serial.print(balance, BIN);
        Serial.print(" | ");
      }
      if (balance != 0) //only send balance command when needed
      {
        payload[0] = y << 1;
        payload[1] = REG_BAL_TIME;
        payload[2] = duration; //balance time limit; renewed every cycle
        BMSUtil::sendData(payload, 3, true);
        delay(2); // original timing
        BMSUtil::getReplyTimed(buff, 4, 5);
        payload[0] = y << 1;
        payload[1] = REG_BAL_CTRL;
        payload[2] = balance; //write balance state to register
        BMSUtil::sendData(payload, 3, true);
        delay(2); // original timing
        BMSUtil::getReplyTimed(buff, 4, 5);
        for (int i = 0; i < 6; i++) if (balance & (1 << i)) CellsBalancing++;
      }
      if (debug == 1)
      {
        Serial.println();
      }
    }
  }
}

uint8_t BMSModuleManager::getBalanceMask(int addr)
{
  return (addr >= 1 && addr <= MAX_MODULE_ADDR) ? balMask[addr] : 0;
}

/*
  After a RESET boards have their faults written due to the hard restart or first time power up, this clears thier faults
*/
void BMSModuleManager::clearFaults()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_ALERT_STATUS;//Alert Status
  payload[2] = 0xFF;//data to cause a reset
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[1] = REG_FAULT_STATUS;//Fault Status
  payload[2] = 0xFF;//data to cause a reset
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  isFaulted = false;
}

void BMSModuleManager::sleepBoards()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_IO_CTRL;//IO ctrl start
  payload[2] = 0x04;//write sleep bit
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
}

void BMSModuleManager::wakeBoards()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_IO_CTRL;//IO ctrl start
  payload[2] = 0x00;//write sleep bit
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);

  payload[0] = 0x7F; //broadcast
  payload[1] = REG_ALERT_STATUS;//Fault Status
  payload[2] = 0x04;//data to cause a reset
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
}

// clearState = true: balancing really stops (from the .ino).
// clearState = false: only switch off briefly for an accurate measurement (getAllVoltTemp);
// the selection of cells is then kept for the hysteresis.
void BMSModuleManager::StopBalancing(bool clearState)
{
  if (clearState)
  {
    for (int x = 0; x <= MAX_MODULE_ADDR; x++) balMask[x] = 0;
    CellsBalancing = 0;
  }
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isOnline())
    {
      modules[x].stopBalance();
    }
  }
}

/////////////////////////////// Measuring ///////////////////////////////

void BMSModuleManager::getAllVoltTemp()
{
  StopBalancing(false); // off during the measurement, keep the selection

  if (getExpectedModules() < 8)
  {
    delay(200);
  }
  else
  {
    delay(50);
  }

  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    // offline modules are skipped here (would only cost time on timeouts);
    // rescan() looks for them and brings them back online.
    if (modules[x].isExisting() && isConfiguredModule(x) && modules[x].isOnline())
    {
      Logger::debug("");
      Logger::debug("Module %i exists. Reading voltage and temperature values", x);
      bool ok = modules[x].readModuleValues();
      modules[x].markReadResult(ok);
      Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
      Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
      Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
    }
  }

  updateStringStatus();

  // Pack voltage: average of the complete strings
  float sumStrings = 0.0f;
  for (int s = 0; s < expectedStrings; s++)
  {
    if (!isStringOnline(s)) continue;
    float sv = 0.0f;
    for (int m = 0; m < modulesPerString; m++) sv += modules[s * modulesPerString + m + 1].getModuleVoltage();
    sumStrings += sv;
  }
  if (stringsOnline > 0)
  {
    packVolt = sumStrings / stringsOnline;
  }
  else if (modulesOnline > 0)
  {
    // no complete string: estimate from the average module voltage
    float sm = 0.0f;
    for (int x = 1; x <= getExpectedModules(); x++) if (modules[x].isOnline()) sm += modules[x].getModuleVoltage();
    packVolt = (sm / modulesOnline) * modulesPerString;
  }
  else
  {
    packVolt = 0.0f;
  }

  if (packVolt > highestPackVolt) highestPackVolt = packVolt;
  if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

  if (digitalRead(11) == LOW) {
    if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
    isFaulted = true;
  }
  else
  {
    if (isFaulted) Logger::info("All modules have exited a faulted state");
    isFaulted = false;
  }

  // Cell min/max over online modules
  HighCellVolt = 0.0f;
  LowCellVolt = 5.0f;
  for (int x = 1; x <= getExpectedModules(); x++)
  {
    if (modules[x].isOnline())
    {
      if (modules[x].getHighCellV() > HighCellVolt) HighCellVolt = modules[x].getHighCellV();
      if (modules[x].getLowCellV() < LowCellVolt) LowCellVolt = modules[x].getLowCellV();
    }
  }
  if (modulesOnline == 0) LowCellVolt = 0.0f; // no data = not safe

  getAvgTemperature(); // also updates lowTemp/highTemp
  for (int x = 1; x <= getExpectedModules(); x++)
  {
    if (modules[x].isOnline() && modules[x].getLowTemp() > -70)
    {
      if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
      if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();
    }
  }
}

int BMSModuleManager::getBalancing()
{
  return CellsBalancing;
}

float BMSModuleManager::getLowCellVolt()
{
  return LowCellVolt;
}

int BMSModuleManager::getNumModules()
{
  return numFoundModules;
}

float BMSModuleManager::getHighCellVolt()
{
  return HighCellVolt;
}

float BMSModuleManager::getPackVoltage()
{
  return packVolt;
}

float BMSModuleManager::getLowVoltage()
{
  return lowestPackVolt;
}

float BMSModuleManager::getHighVoltage()
{
  return highestPackVolt;
}

void BMSModuleManager::setBatteryID(int id)
{
  batteryID = id;
}

void BMSModuleManager::setPstrings(int Pstrings)
{
  setTopology(settings.Scells, Pstrings);
}

void BMSModuleManager::setSensors(int sensor, float Ignore)
{
  // apply to all addresses, so modules found later also get the right setting
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    modules[x].settempsensor(sensor);
    modules[x].setIgnoreCell(Ignore);
  }
}

float BMSModuleManager::getAvgTemperature()
{
  float avg = 0.0f;
  int n = 0;
  lowTemp = 999.0f;
  highTemp = -999.0f;
  for (int x = 1; x <= getExpectedModules(); x++)
  {
    if (modules[x].isOnline() && modules[x].getAvgTemp() > -70)
    {
      avg += modules[x].getAvgTemp();
      n++;
      if (modules[x].getHighTemp() > highTemp) highTemp = modules[x].getHighTemp();
      if (modules[x].getLowTemp() < lowTemp) lowTemp = modules[x].getLowTemp();
    }
  }
  if (n == 0)
  {
    lowTemp = 0.0f;
    highTemp = 0.0f;
    return 0.0f;
  }
  return avg / (float)n;
}

float BMSModuleManager::getHighTemperature()
{
  return highTemp;
}

float BMSModuleManager::getLowTemperature()
{
  return lowTemp;
}

float BMSModuleManager::getAvgCellVolt()
{
  float avg = 0.0f;
  int n = 0;
  for (int x = 1; x <= getExpectedModules(); x++)
  {
    if (modules[x].isOnline())
    {
      avg += modules[x].getAverageV();
      n++;
    }
  }
  if (n == 0) return 0.0f;
  return avg / (float)n;
}

void BMSModuleManager::printPackSummary()
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i    Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules,
                  getPackVoltage(), getAvgCellVolt(), getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      Logger::console("                               Module #%i", y);

      Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                      modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());
      if (faults > 0)
      {
        Logger::console("  MODULE IS FAULTED:");
        if (faults & 1)
        {
          SERIALCONSOLE.print("    Overvoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 6; i++)
          {
            if (COV & (1 << i))
            {
              SERIALCONSOLE.print(i + 1);
              SERIALCONSOLE.print(" ");
            }
          }
          SERIALCONSOLE.println();
        }
        if (faults & 2)
        {
          SERIALCONSOLE.print("    Undervoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 6; i++)
          {
            if (CUV & (1 << i))
            {
              SERIALCONSOLE.print(i + 1);
              SERIALCONSOLE.print(" ");
            }
          }
          SERIALCONSOLE.println();
        }
        if (faults & 4)
        {
          Logger::console("    CRC error in received packet");
        }
        if (faults & 8)
        {
          Logger::console("    Power on reset has occurred");
        }
        if (faults & 0x10)
        {
          Logger::console("    Test fault active");
        }
        if (faults & 0x20)
        {
          Logger::console("    Internal registers inconsistent");
        }
      }
      if (alerts > 0)
      {
        Logger::console("  MODULE HAS ALERTS:");
        if (alerts & 1)
        {
          Logger::console("    Over temperature on TS1");
        }
        if (alerts & 2)
        {
          Logger::console("    Over temperature on TS2");
        }
        if (alerts & 4)
        {
          Logger::console("    Sleep mode active");
        }
        if (alerts & 8)
        {
          Logger::console("    Thermal shutdown active");
        }
        if (alerts & 0x10)
        {
          Logger::console("    Test Alert");
        }
        if (alerts & 0x20)
        {
          Logger::console("    OTP EPROM Uncorrectable Error");
        }
        if (alerts & 0x40)
        {
          Logger::console("    GROUP3 Regs Invalid");
        }
        if (alerts & 0x80)
        {
          Logger::console("    Address not registered");
        }
      }
      if (faults > 0 || alerts > 0) SERIALCONSOLE.println();
    }
  }
}

void BMSModuleManager::printPackDetails(int digits)
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;
  int cellNum = 0;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i Cells: %i Strings: %i  Voltage: %fV   Avg Cell Voltage: %fV  Low Cell Voltage: %fV   High Cell Voltage: %fV Delta Voltage: %zmV   Avg Temp: %fC ", numFoundModules, seriescells(),
                  Pstring, getPackVoltage(), getAvgCellVolt(), LowCellVolt, HighCellVolt, (HighCellVolt - LowCellVolt) * 1000, getAvgTemperature());
  SERIALCONSOLE.print("Strings online: ");
  SERIALCONSOLE.print(stringsOnline);
  SERIALCONSOLE.print("/");
  SERIALCONSOLE.print(expectedStrings);
  SERIALCONSOLE.print("  Modules online: ");
  SERIALCONSOLE.print(modulesOnline);
  SERIALCONSOLE.print("/");
  SERIALCONSOLE.print(getExpectedModules());
  SERIALCONSOLE.print("  Cells balancing: ");
  SERIALCONSOLE.print(CellsBalancing);
  SERIALCONSOLE.println("  (* = cell is being bled)");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      SERIALCONSOLE.print("Module #");
      SERIALCONSOLE.print(y);
      if (y < 10) SERIALCONSOLE.print(" ");
      if (isConfiguredModule(y))
      {
        SERIALCONSOLE.print(" S");
        SERIALCONSOLE.print(stringOfModule(y) + 1);
      }
      else
      {
        SERIALCONSOLE.print(" --");
      }
      SERIALCONSOLE.print(modules[y].isOnline() ? "        " : " OFFLINE");
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(modules[y].getModuleVoltage(), digits);
      SERIALCONSOLE.print("V");
      for (int i = 0; i < 6; i++)
      {
        if (cellNum < 10) SERIALCONSOLE.print(" ");
        SERIALCONSOLE.print("  Cell");
        SERIALCONSOLE.print(cellNum++);
        SERIALCONSOLE.print(": ");
        SERIALCONSOLE.print(modules[y].getCellVoltage(i), digits);
        SERIALCONSOLE.print((balMask[y] & (1 << i)) ? "V*" : "V ");
      }
      SERIALCONSOLE.print("  Neg Term Temp: ");
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      SERIALCONSOLE.print("C  Pos Term Temp: ");
      SERIALCONSOLE.print(modules[y].getTemperature(1));
      SERIALCONSOLE.println("C");

    }
  }
}

void BMSModuleManager::printAllCSV(unsigned long timestamp, float current, int SOC, int delim)
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      SERIALCONSOLE.print(timestamp);
      if (delim == 1)
      {
        SERIALCONSOLE.print(" ");
      }
      else
      {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(current, 0);
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(SOC);
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(y);
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      for (int i = 0; i < 8; i++)
      {
        SERIALCONSOLE.print(modules[y].getCellVoltage(i));
        if (delim == 1)       {
          SERIALCONSOLE.print(" ");
        }       else       {
          SERIALCONSOLE.print(",");
        }
      }
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(modules[y].getTemperature(1));
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(modules[y].getTemperature(2));
      SERIALCONSOLE.println();
    }
  }
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      Serial2.print(timestamp);
      if (delim == 1)
      {
        Serial2.print(" ");
      }
      else
      {
        Serial2.print(",");
      }
      Serial2.print(current, 0);
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      Serial2.print(SOC);
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      Serial2.print(y);
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      for (int i = 0; i < 8; i++)
      {
        Serial2.print(modules[y].getCellVoltage(i));
        if (delim == 1)       {
          Serial2.print(" ");
        }       else       {
          Serial2.print(",");
        }
      }
      Serial2.print(modules[y].getTemperature(0));
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      Serial2.print(modules[y].getTemperature(1));
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      Serial2.print(modules[y].getTemperature(2));
      Serial2.println();
    }
  }
}

uint16_t BMSModuleManager::getcellvolt(int modid, int cellid)
{
  return (modules[modid].getCellVoltage(cellid) * 1000);
}

uint16_t BMSModuleManager::gettemp(int modid, int tempid)
{
  return (modules[modid].getTemperature(tempid));
}
