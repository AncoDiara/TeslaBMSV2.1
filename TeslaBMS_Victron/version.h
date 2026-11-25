#pragma once
/*
  TeslaBMS_Victron - firmware version, shown by VenusOS as v<FW_YEAR>.<FW_BUILD>
  (CAN 0x35F, byte 2 = year, byte 3 = build number).

  FW_BUILD is increased by 1 with every commit that changes the FIRMWARE (sketch files),
  so that every firmware version can be traced. Above 255 it wraps around (only the
  lowest byte is sent). Changes to only the dashboard (nodered/), the Cerbo scripts
  (cerbo/) or documentation do NOT raise FW_BUILD; the dashboard has its own version
  (TESLABMS_FLOW_VERSION in nodered/tesla-bms-dashboard.json).
  The GitHub workflow refuses a push that changes the firmware without raising FW_BUILD.
*/
#define FW_YEAR   26   // year of the commit (2026)
#define FW_BUILD 37   // firmware build number (raise with every firmware change)
