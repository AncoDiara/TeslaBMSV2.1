#pragma once
/*
  TeslaBMS_Victron - firmware version, shown by VenusOS as v<FW_YEAR>.<FW_BUILD>
  (CAN 0x35F, byte 2 = year, byte 3 = build number).

  FW_BUILD is increased by 1 with EVERY commit to GitHub, so that every version can be
  traced. Above 255 it wraps around (only the lowest byte is sent).
  Did you change something yourself and commit it? Then raise FW_BUILD by 1 as well.
*/
#define FW_YEAR   26   // year of the commit (2026)
#define FW_BUILD  9    // commit number of TeslaBMS_Victron
