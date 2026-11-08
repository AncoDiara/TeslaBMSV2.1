# TeslaBMSV2

This software is designed to run on the SimpBMS.

User Manual that covers some software functions https://github.com/tomdebree/SimpBMS 

## TeslaBMS_Victron ("Claude Tesla BMS")

The folder [`TeslaBMS_Victron`](TeslaBMS_Victron/) contains an adapted version for Tesla 6S modules
on a Victron VenusOS system: fixed/auto-detected topology with strings (24 V / 48 V),
robust module detection, alarm on a missing module while CVL/CCL/DCL keep being sent, tight
balancing per string, and "modules online/offline" per string in VenusOS.
See [TeslaBMS_Victron/README.md](TeslaBMS_Victron/README.md). The original TeslaBMSV2 code above is unchanged.
