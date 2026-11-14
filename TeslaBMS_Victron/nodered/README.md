# Node-RED dashboard (FlowFuse Dashboard 2.0) for TeslaBMS_Victron

Connect the Teensy's USB to a USB port of the Cerbo GX (or another Venus OS Large device).
The Teensy then sends one JSON line every 5 seconds with all cell voltages, balancing, limits and alarms.

## 1. Teensy: switch JSON output on
Serial monitor (115200 baud): `s` → `m` → `j` (JSON output **On**) → `q` to save.
While a menu is open the JSON pauses; the menu closes by itself after 2 minutes without input.

## 2. Node-RED: palette
Install via *Manage palette* (if not already present):
- `@flowfuse/node-red-dashboard` (Dashboard 2.0)
- `node-red-node-serialport`

## 3. Import the flow
Import `tesla-bms-dashboard.json`. Deploy. The dashboard is at `/dashboard/tesla-bms`.
Already have a Dashboard 2.0 base or theme? Point the page *Tesla BMS* at your own base/theme
and delete the imported ones (Dashboard 2.0 uses one base).

## 4. Port and VE_SERVICE=ignore (automatic)
At start and every 60 s the flow:
- looks up the Teensy via `/dev/serial/by-id/` (name contains `Teensy`) and sets that port
  through the *serial control* node (stable name, even if it becomes ttyACM1);
- writes a udev rule so that VenusOS serial-starter leaves the Teensy alone:
  `/etc/udev/rules.d/zz-teslabms-teensy.rules` with `ENV{VE_SERVICE}="ignore"` for USB-ID `16c0:0483`;
- adds a line to `/data/rc.local`, so the rule comes back after a firmware update
  (`/data` survives updates);
- reconnects if no data has arrived for 60 s.

The status under the node *Set port* shows the result. If it says `noperm`, Node-RED is not
allowed to write to `/etc` or `/data`: then run `install-teensy-ignore.sh` once via SSH as root.

Check (SSH): `udevadm info /dev/ttyACM0 | grep VE_SERVICE` should show `VE_SERVICE=ignore`.

## Power
In this setup the Teensy is powered only from USB: the BMS runs as long as the GX is on.
When the GX restarts the Teensy restarts as well and counts the modules again.

## JSON fields
`fw` version, `st` status, `cvl`/`ccl`/`dcl`, `v` battery voltage, `i` current, `soc`,
`so`/`se` strings online/expected, `mo`/`me` modules, `bal` cells balancing, `lt` duration of
a read round (ms), `lo`/`hi` + `loid`/`hiid` lowest/highest cell, `tlo`/`thi` temperatures,
`ovl` overvoltage latch, `mw`/`ma` module warning/alarm, `alm`/`wrn` alarm bytes of 0x35A,
`mod[]` per module: `a` address, `s` string, `on`, `v`, `c[6]` cells, `t[2]` temperatures, `b` balance mask.
