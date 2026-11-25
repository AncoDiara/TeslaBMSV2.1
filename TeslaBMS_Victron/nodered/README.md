# Node-RED dashboard (FlowFuse Dashboard 2.0) for TeslaBMS_Victron

Connect the Teensy's USB to a USB port of the Cerbo GX (or another Venus OS Large device).
The Teensy then sends one JSON line every 5 seconds with all cell voltages, balancing, limits and alarms.

## 1. Teensy: JSON output (automatic)
The flow switches the JSON output on by itself: if no JSON arrives for 20 s it sends `J` to the
Teensy (JSON on, saved in the Teensy). `T` switches back to text output. By hand: serial monitor
(115200 baud) `s` → `m` → `j`. While a menu is open the JSON pauses; the menu closes by itself
after 2 minutes without input.

## 2. Node-RED: palette
Install via *Manage palette* (if not already present):
- `@flowfuse/node-red-dashboard` (Dashboard 2.0)
- `node-red-node-serialport`

## 3. Import the flow
Import `tesla-bms-dashboard.json`. Deploy. The dashboard is at `/dashboard/tesla-bms`.
Already have a Dashboard 2.0 base or theme? Point the page *Tesla BMS* at your own base/theme
and delete the imported ones (Dashboard 2.0 uses one base).

## 4. Port and VE_SERVICE=ignore (automatic)
At start and every 20 s the flow:
- looks up the Teensys via `/dev/serial/by-id/` (name contains `Teensy`) and assigns them
  (node *Assign Teensy ports*, see below) through the *serial control* nodes (stable names, even if
  they become ttyACM1);
- writes a udev rule so that VenusOS serial-starter leaves the Teensy alone:
  `/etc/udev/rules.d/zz-teslabms-teensy.rules` with `ENV{VE_SERVICE}="ignore"` for USB-ID `16c0:0483`;
- adds a line to `/data/rc.local`, so the rule comes back after a firmware update
  (`/data` survives updates);
- reconnects if no data has arrived for 60 s.

The status under the node *Assign Teensy ports* shows the result. If it says `noperm`, Node-RED is not
allowed to write to `/etc` or `/data`: then run `install-teensy-ignore.sh` once via SSH as root.

Check (SSH): `udevadm info /dev/ttyACM0 | grep VE_SERVICE` should show `VE_SERVICE=ignore`.

## Second Teensy: test bench (tab *Tesla BMS Test*)
A second Teensy (for example a Teensy 4.0 with one or two modules, without CAN or charger) can be
connected to another USB port of the GX to test modules / CMUs. It gets its own tab and dashboard
page **Tesla BMS Test** (`/dashboard/tesla-bms-test`) with the same overview and history, plus
firmware update and balancing for that Teensy. It sends nothing to Victron (no CAN connected).

Which Teensy is which:
- A Teensy is recognised by its **USB serial number** (`/dev/serial/by-id/usb-Teensyduino_USB_Serial_<number>-if00`),
  not by the USB socket, so swapping sockets changes nothing.
- The first time with more than one Teensy, both are opened; as soon as both send data the one with
  the **most modules** becomes the main battery (tab *Tesla BMS*), the other the test bench.
- The assignment is saved in `/data/home/nodered/.node-red/teslabms-teensy-roles.json`. After a
  restart of the GX it is used straight away, and the main tab never takes the test Teensy (also not
  when the main Teensy is unplugged).
- The history charts start only once the assignment is certain, so no data of the other Teensy ends up in them.
- Each tab opens its **own fixed port name**, a symlink in the Node-RED folder:
  `teslabms-tty-main` and `teslabms-tty-test`, pointed at the right Teensy by the assignment
  (`ln -sfn`). The serial node of Node-RED shares connections by port name, so with these
  names two tabs can never end up reading the same Teensy.
- Safety net: if the saved assignment is wrong (the test Teensy reports clearly more modules than
  the main one), it is corrected automatically.
- The saved main Teensy gone for **more than 2 minutes** (replaced, broken): another Teensy takes over the main
  role (the one with the most modules; with one Teensy: that one) and this is saved. A short absence (restart,
  firmware update of the main Teensy) never swaps the roles.
- Fix the main Teensy by hand: tab variable `TESLABMS_MAIN_TEENSY` (tab *Tesla BMS*) = its USB serial number
  (the number in `/dev/serial/by-id/usb-Teensyduino_USB_Serial_<number>-if00`); empty = automatic.
- Assign again from scratch: delete the roles file via SSH and restart Node-RED.

Firmware updates always flash exactly the Teensy of that tab (`teensy_flash.py --port ...`), and
never two at the same time.

## SolarEdge (tab *SolarEdge*): two battery systems behind one grid meter
For a SolarEdge StorEdge inverter (e.g. SE10K-RWS) next to the Victron system, with the Victron
energy meter as the only grid meter. Node-RED on the GX coordinates both:

- every 5 s: grid power (L1+L2+L3) and Victron battery power, either from this GX (`com.victronenergy.system`)
  or from **another GX over MQTT** (setting *grid + battery of*: that GX has the grid meter; enable
  *MQTT on LAN (insecure)* on it, port 1883; the portal ID is found automatically and a keep alive is sent
  every 30 s), and the SolarEdge battery power over Modbus TCP;
  the tab variable `SOLAREDGE_GRID_GX` (now `192.168.1.10`) is the default IP of that GX as long as no
  settings were saved on the page (empty = this GX);
- **T = grid − Victron battery − SolarEdge battery** (battery power + = charging) is what all storage
  together has to deliver (T > 0) or may absorb (T < 0);
- the SolarEdge takes the set **share** of T: discharge (command mode 4, discharge limit) or charge from its
  own solar (mode 2, charge limit); Victron ESS regulates the rest to zero. **Both batteries always charge
  together or discharge together**, never one against the other (that would only cost conversion losses);
- split (setting): **fixed share**, or **same SOC**: the share follows the capacities of both batteries
  (e.g. 30 and 120 kWh -> 80 %) and shifts with the SOC difference (10 % difference = 20 % points), so the
  fuller battery discharges more / charges less until both have the same state of charge, then they go on
  together. SOC of system 1 from the GX (dbus or MQTT), SOC of the SolarEdge battery from the inverter;
  without both: capacity share;
- every command has a timeout of 60 s; if Node-RED stops, the inverter falls back to "charge from solar";
- settings on the dashboard page **SolarEdge** (inverter IP, on/off, share, max charge/discharge), saved in
  `teslabms-solaredge.json` in the Node-RED folder. With control off it only shows the status.

Needed: Modbus TCP enabled on the inverter (SetApp → Site Communication → Modbus TCP, port 1502), the
Tesla battery on the SolarEdge with the LG RESU LV CAN protocol (see the firmware README). Script:
`solaredge_ctl.py` (pure Python, fetched with the update scripts; also usable by hand:
`python3 /data/teslabms/solaredge_ctl.py --host <ip> status`).

## Page *Overzicht* (first page): both systems, grid and solar
Retro gauges, every 2 s:
- **Net** (system 1's grid meter, per phase), **Zon** (PV of system 1: DC MPPT + PV inverters on AC, plus the
  SolarEdge's own PV = its AC output + its battery charging), **Huis** (= grid + solar − charging of both
  batteries) and **Opslag samen**. The red needle is now, the dashed needle(s) the **peak of today**
  (grid: import and export; storage: charge and discharge), with value and time.
- Per system: SOC, power (with daily peaks) and details; system 2 also the pack voltage and the BMS values.
- **Vandaag**: kWh counters (solar, grid import/export, house, charged/discharged).
- Peaks and counters reset at 00:00 and are saved every 10 min (`teslabms-overview.json`), a restart keeps them.
- 24 h history: SOC of both systems, grid and solar (`teslabms-overview-history.json`).

System 1 comes over MQTT, so the SolarEdge tab must use "grid + battery of: another GX (MQTT)" (tab variable
`SOLAREDGE_GRID_GX`); it also subscribes to the battery voltage/current/temperature and PV of that GX.

## Power
In this setup the Teensy is powered only from USB: the BMS runs as long as the GX is on.
When the GX restarts the Teensy restarts as well and counts the modules again.

## CVL tile and DVCC limit
The CVL tile shows the CVL calculated by the BMS (Teensy). Below it: the
**DVCC "Limit charge voltage"** setting of the GX (Settings → DVCC), read every 30 s from dbus
(`com.victronenergy.settings /Settings/SystemSetup/MaxChargeVoltage`). `off` = limit not set.

## JSON fields
`fw` version, `st` status, `cvl`/`ccl`/`dcl`, `v` battery voltage, `i` current, `soc`,
`so`/`se` strings online/expected, `mo`/`me` modules, `bal` cells balancing, `lt` duration of
a read round (ms), `lo`/`hi` + `loid`/`hiid` lowest/highest cell, `tlo`/`thi` temperatures,
`ovl` overvoltage latch, `mw`/`ma` module warning/alarm, `alm`/`wrn` alarm bytes of 0x35A,
`mod[]` per module: `a` address, `s` string, `on`, `v`, `c[6]` cells, `t[2]` temperatures, `b` balance mask.
`sc` cells in series per string, `ps` parallel strings, `er` 1 = EEPROM was reset to the defaults
(Teensy 4 after a firmware update) and no setting has been sent since.

### Settings guard (restore after a Teensy 4 firmware update)
A Teensy 4.0 loses its EEPROM settings at every upload. The function *Settings guard* in each tab
saves the settings of each Teensy (by USB serial number) in
`/data/home/nodered/.node-red/teslabms-teensy-settings.json`: cells in series, CAN protocol, CAN
speed and balancing start/stop. When a Teensy reports `"er": 1`, the guard sends them back once
(at most once a minute). **Maintenance & updates** has buttons **24 V (6S)** / **48 V (12S)**
(double click) to set the cells in series by hand.

## Updates from the dashboard (firmware and the dashboard itself)

The collapsible block **Maintenance & updates** (closed by default, click the title) shows the running Teensy firmware, the dashboard version and the
latest version on GitHub (checked at start and every 6 hours, or with **Check**).

- **Update Teensy firmware** (click twice within 10 s): fetches the `.hex` of the latest GitHub release
  (built by GitHub Actions with the official Teensy core, SHA256 checked) and flashes it via USB.
  The BMS is off for a few seconds; Victron stops charging/discharging and continues afterwards.
- **Update dashboard** (click twice within 10 s): replaces only the *Tesla BMS* tab with the latest
  version (backup of the flows file first), then Node-RED restarts. Reload the page after ~30 s.
  Other flows and an existing Dashboard 2.0 base are left untouched.

The scripts (`teensy_flash.py`, `flow_update.py`) are kept in `/data/teslabms/` (or
`~nodered/teslabms/` if that folder is not writable) and are refreshed from the latest release at every check.

**Node-RED runs as user `nodered` on Venus OS, not as root.** Therefore run
[`install-teensy-ignore.sh`](install-teensy-ignore.sh) once via SSH as root. It gives Node-RED access
to the Teensy bootloader (needed for firmware updates) and to `/data/teslabms`:
```bash
ssh root@<gx-ip> 'sh -s' < install-teensy-ignore.sh
```
The dashboard update restarts Node-RED by stopping its own process; the Venus service supervisor
starts it again.

### First time (from an older dashboard without the update block)
Via SSH as root on the GX:
```bash
mkdir -p /data/teslabms && cd /data/teslabms && python3 -c "import urllib.request as u
for f in ('teensy_flash.py','flow_update.py'):
    open(f,'wb').write(u.urlopen(u.Request('https://github.com/AncoDiara/TeslaBMSV2.1/releases/latest/download/'+f,headers={'User-Agent':'teslabms'})).read())"
python3 flow_update.py --latest
```
This replaces the old Tesla BMS tab (also an old version with other node IDs) and restarts Node-RED.

### By hand via SSH
```bash
python3 /data/teslabms/teensy_flash.py --check            # versions
python3 /data/teslabms/teensy_flash.py --latest           # flash the Teensy (asks for confirmation)
python3 /data/teslabms/flow_update.py --latest            # update the dashboard (asks for confirmation)
```
If flashing is interrupted, the Teensy stays in its bootloader (BMS off): run it again, or press the
button on the Teensy and use the Teensy Loader / Arduino IDE on a PC.
