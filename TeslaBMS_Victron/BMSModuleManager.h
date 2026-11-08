#pragma once
#include "config.h"
#include "BMSModule.h"

class BMSModuleManager
{
  public:
    BMSModuleManager();
    int seriescells();
    void clearmodules();
    void StopBalancing(bool clearState = true);
    uint8_t getBalanceMask(int addr);        // NEW: bit 0..5 = cell 1..6 is being bled
    void balanceCells(int duration, int debug);
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void readSetpoints();
    void setBatteryID(int id);
    void setPstrings(int Pstrings);
    void setUnderVolt(float newVal);
    void setOverVolt(float newVal);
    void setOverTemp(float newVal);
    void setBalanceV(float newVal);
    void setBalanceHyst(float newVal);
    void setSensors(int sensor, float Ignore);
    float getPackVoltage();
    float getAvgTemperature();
    float getHighTemperature();
    float getLowTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    float getHighVoltage();
    float getLowVoltage();
    int getBalancing();
    uint16_t getcellvolt(int modid, int cellid);
    uint16_t gettemp(int modid, int tempid);
    /*
      void processCANMsg(CAN_FRAME &frame);
    */
    void printAllCSV(unsigned long timestamp,float current, int SOC, int delim);
    void printPackSummary();
    void printPackDetails(int digits);
    int getNumModules();

    // NEW: topology / strings
    void setTopology(int scellsPerString, int pstrings);
    void startupDetect();
    void rescan();
    void printTopology();
    void updateStringStatus();
    bool probeAddress(int addr, int attempts);
    bool isConfiguredModule(int addr);
    bool isStringOnline(int s);
    bool isModuleOnline(int addr);
    int stringOfModule(int addr);
    int getModulesPerString();
    int getStringsExpected();
    int getExpectedModules();
    int getStringsOnline();
    int getStringsOffline();
    int getModulesOnline();
    int getExtraModules();

    // NEW: location of the extremes (for CAN 0x374-0x377). addr = module address (1..),
    // num = cell 1..6 or temperature sensor 1..2. 0 = unknown.
    uint8_t lowCellAddr, lowCellNum, highCellAddr, highCellNum;
    uint8_t lowTempAddr, lowTempNum, highTempAddr, highTempNum;

  private:
    float packVolt;                         // All modules added together
    int Pstring;
    float LowCellVolt;
    float HighCellVolt;
    float lowestPackVolt;
    float highestPackVolt;
    float lowestPackTemp;
    float highestPackTemp;
    float highTemp;
    float lowTemp;
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
    int batteryID;
    int numFoundModules;                    // The number of modules that seem to exist
    bool isFaulted;
    int spack;
    int CellsBalancing;                     // number of cells being balanced
    uint8_t balMask[MAX_MODULE_ADDR + 1];   // NEW: current balance mask per module
    int modulesPerString;                   // NEW: Scells / 6
    int expectedStrings;                    // NEW: Pstrings
    int stringsOnline;                      // NEW
    int modulesOnline;                      // NEW (only modules within the topology)
    int extraModules;                       // NEW: found modules outside the topology
    /*
      void sendBatterySummary();
      void sendModuleSummary(int module);
      void sendCellDetails(int module, int cell);
    */

};
