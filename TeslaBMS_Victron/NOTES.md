# Handover notes (2026-11-25)

State at release **fw-v26.40_flow-v26.51** (master). Written by the cloud session for the session on
the laptop (`/home/anco/TeslaBMS`), which takes over from here.

## Installation (system 2, Cerbo X "einstein", 192.168.1.116)
- Teensy 4.0 (main BMS, replaced the dead Teensy 3.2), 24 modules = 12 strings of 12S (2 modules per
  string, ~230 Ah per string). USB to the Cerbo; serial symlink `/dev/teslabms-tty-main`.
- CAN: BMS-Can 500 kbit/s, LG RESU LV protocol (`P1`) to a SolarEdge SE-RWS. The SolarEdge is the
  only charger/discharger of this battery. Modbus TCP to the SolarEdge: 192.168.1.178:1502.
- No shunt / BMV / Lynx on this battery. The only battery service on the Cerbo is the Teensy itself
  (`com.victronenergy.battery.socketcan_can0`, "CAN-bus BMS battery").
- System 1 (GX1 192.168.1.10, ESS, ~30 kWh, VM-3P75CT) is only read over MQTT (tab SolarEdge, page Overzicht).

## Done (and why)
| Version | What | Why |
|---|---|---|
| fw 26.36 / v26.47 | T4 CAN in polling mode (no FlexCAN_T4 interrupts) | T4 hung as soon as other devices acknowledged on the bus |
| fw 26.36 / v26.47 | Default 12S, `C<cells>`, JSON `sc`/`ps`/`er`, settings guard, 24/48 V buttons | A Teensy 4 upload erases its (emulated) EEPROM; the dashboard restores C/P/K/B per USB serial (`teslabms-teensy-settings.json`) |
| fw 26.37 / v26.48 (laptop session) | Direct command closes an open menu; `B<start>,<stop>,<from mV>`; patient JSON watchdog; one long-running dbus reader | Periodic J/S kept a menu open (no JSON); Cerbo CPU was at 100 % (dbus polling + crashing `dbus-mrdk`, now removed) |
| fw 26.38 / v26.49 | JSON `up` (uptime s) and `rr` (reason of last start); header shows uptime, restarts in 24 h in red | To see whether the T4 still restarts. Note: on a T4 a start after flashing shows as `power` |
| fw 26.39 / v26.50 | Cold charge curve per string by the COLDEST sensor: UnderT (−10 °C) 0 · −5 °C 2.5 A · 0 °C 5 A · 5 °C 12 A · ChargeT (10 °C) full. JSON `cold`, CCL tile shows it. EEPROM 0x20: ChargeTSetpoint 0 → 10 °C once | NCA: lithium plating when charging below ~10 °C. Old code allowed 90 % current at −1 °C. Total 12 strings: 60 A at 0 °C (agreed with Anco). Discharge not limited. Tested on the host: linear, no jump at 10 °C |
| fw 26.40 / v26.51 | Measured current → 0 A after 30 s without a new value; `I<A>` = current for information; JSON `is` (0 none, 1 measured, 2 info); flow sends `I` = SolarEdge `bat_p` / pack voltage when there is no shunt | 7.0 A stood frozen for hours (one stray value, never cleared, also passed on over CAN). SOC deliberately stays from the cell voltage (no coulomb counting on an estimate); a real shunt always wins |
| fw 26.41 / v26.51 (laptop session) | LG RESU mode also sends the battery name (0x35E "Claude T" + 0x370/0x371 "Claude Tesla BMS") | Venus OS showed "CAN-bus BMS battery" in LG mode. Tested: Venus shows "Claude Tesla BMS", SolarEdge keeps charging, keep-alive OK, no CAN errors. The CustomName set in between on the Cerbo is cleared again |
| CI | Publish step retries 4x and removes a half-finished draft | GitHub gave 500/502 on release uploads |

## Open
1. **Install and verify v26.40 / v26.51** on the Cerbo (dashboard first, then firmware). Check: Current tile
   "from SolarEdge (W / V)" with the sign of the SolarEdge (− = discharging); CCL tile "charging limited:
   cold" below 10 °C; header back to 12S + SolarEdge after the flash (settings guard).
2. **Stray `S` after a restart** (`sx:1`, SOC 4 %, 7 A seen once right after flashing fw 26.38): origin
   not found. The only sender of `S` is *shunt SOC -> Teensy*, which finds no shunt. Since v26.40 the
   effect is limited (current 0 A after 30 s, external SOC only used < 60 s), but the cause is unknown.
   Idea: log every line Node-RED writes to the Teensy for a while, or have the firmware report the last
   received direct command in the JSON.
3. **T4 stability**: stable since the Cerbo CPU fix (fw 26.37). Watch the header for restarts
   (`rr` watchdog/lockup) now that CAN is connected again. Earlier: bus-off counter at 718 (stood still).
4. **Cell balance**: Pk3 C3 (~3.63 V) and Pk9 C3 (~3.70 V) lowest, Pk4 C5 highest, spread ~120 mV.
   Anco planned to swap packs 3 and 5.
5. **SOC**: from the cell voltage (OCV table, filtered). If a Lynx/SmartShunt is added later it is picked up
   automatically (`TESLABMS_SOC_SOURCE=auto`); then sync its SOC first.
6. **CAN keep-alive panel** (LG mode) shows 0x305; the Cerbo on the same bus also sends 0x305/0x307 "VIC".
   The SolarEdge itself may only send other IDs (e.g. 0x308 / 0x71F): check the received-ID table, so the
   keep-alive really means "SolarEdge alive".
7. **SolarEdge ↔ system 1 coordination** (same SOC or fixed share, never charge one while the other
   discharges): built in the SolarEdge tab, needs long-term observation.
8. Optional: README line "works with SolarEdge RWS (LG RESU LV protocol)".

## Working notes
- The laptop session cannot push to GitHub yet: `gh auth login` + `gh auth setup-git` once, or hand over
  a patch (`git format-patch`). Every push to master that changes the firmware must raise `FW_BUILD`
  (version.h), a flow change must raise `TESLABMS_FLOW_VERSION`; CI builds T32 + T40 and publishes the release.
- Teensy 4 upload = EEPROM erased: expected; the settings guard restores the settings within a minute.
- Venus OS: BusyBox (no `timeout`, no `head -c`); stop Node-RED (`svc -d /service/node-red-venus`) before
  reading the serial port directly.
