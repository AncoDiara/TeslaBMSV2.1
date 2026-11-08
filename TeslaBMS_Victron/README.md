# TeslaBMS_Victron – "Claude Tesla BMS"

A version of [TeslaBMSV2](../TeslaBMSV2.ino) (SimpBMS by Simp ECO Engineering / Tom de Bree)
adapted for **Tesla Model S/X 6S modules on a Victron VenusOS system** (GX device, DVCC, CAN-bus BMS).

The adaptations were written together with Claude (Anthropic) and tested on hardware by [AncoDiara](https://github.com/AncoDiara).
The original TeslaBMSV2 code in this repository is left unchanged.

> **Warning – use at your own risk.** A BMS protects a battery and is safety-critical.
> This version has been tested in a software simulation of the Tesla module chain and on a
> 24 V system with 5–6 modules in parallel. Check all settings and alarms on your own system
> before relying on it, and always use independent protection (fuse, breaker).

Setting up a Windows laptop to always fetch and flash the latest version: see **[WINDOWS-SETUP.md](WINDOWS-SETUP.md)**.

## Hardware

- SimpBMS V2 board with **Teensy 3.2** (Teensyduino; libraries FlexCAN and ADC come with Teensyduino)
- Tesla Model S/X battery modules (6 cells in series, 6S74P)
- Victron GX device (VenusOS) on CAN-bus (VE.Can / BMS-Can, 500 kbit/s)

No external "Filters" library is needed anymore: the low-pass filter for the current measurement is built in (`LowPassFilter.h`).
There are several libraries called "Filters", and installing the wrong one gave the error
`'FilterOnePole' does not name a type`.

## Topology: strings

Modules are grouped into **strings** (modules in series), and the strings are connected in parallel.

| System | Scells | Modules per string | Pstrings |
|---|---|---|---|
| 24 V | 6 | 1 | number of modules in parallel |
| 48 V | 12 | 2 | number of strings in parallel |

- Addresses are assigned in **order of the communication chain**: address 1..M = string 1, M+1..2M = string 2, etc.
  So wire the comm chain string by string.
- **Auto-detect (default on):** at every start the modules are renumbered in chain order and counted.
  Number of strings = modules found / modules per string (rounded up, so a half 48 V string shows up as offline).
- Auto-detect off (menu `m` → `a`): fixed number via menu `b` → `0`. Missing modules then give an alarm already at startup.
- The Tesla comm bus is a daisy chain: if one module board fails, the modules after it are unreachable too.

## Behaviour towards VenusOS

| CAN ID | Content |
|---|---|
| 0x351 | CVL (controlled, see below), CCL, DCL (per string × strings online), discharge voltage |
| 0x355 | SOC, SOH, SOC 0.01 % (fixed: was a factor 10 too low) |
| 0x356 | Pack voltage (average of the complete strings), current, temperature |
| 0x35A | Alarms/warnings, incl. **BMS internal** (byte 2/6, 0x40) when a string is missing |
| 0x35E | Manufacturer: `Claude` |
| 0x35F | Firmware version = compile date (byte 2 year, byte 3 week number), capacity in Ah; sent every second |
| 0x370 + 0x371 | Battery name: `Claude Tesla BMS` |
| 0x372 | **Strings** online / offline (VenusOS shows 1 string as 1 "module"), blocking charge/discharge |
| 0x373 | Min/max cell voltage and temperature |
| 0x374 / 0x375 | **Which cell** has the lowest / highest voltage, e.g. `Pk3 C5` = pack (module) 3, cell 5; sent every second |
| 0x376 / 0x377 | Which temperature sensor is the coldest / warmest, e.g. `Pk2 T1` |
| 0x379 | Installed capacity (Ah per string × number of strings) |

**Missing module/string during operation:** the BMS does *not* go to Error; CVL/CCL/DCL keep being sent.

- immediately: warning "BMS internal"
- after `ModuleAlarmDelay` seconds (default 60): alarm "BMS internal"
- CCL/DCL are scaled with strings online / strings total (can be switched off: menu `m` → `s`)
- every 30 s the missing modules are searched for again; a module that lost its address after a power interruption gets its old address (= the right string) back

## Charge control: CVL, CCL and DCL (Victron style)

**CCL/DCL** are set **per string** (per pack at 24 V):

- default 25 A charge and 25 A discharge per string (menu `e` → `3`, menu `b` → `8`)
- sent to Victron: value per string × number of strings online (with 5 packs: 125 A)
- the CCL is **not** tapered by cell voltage. It only goes to **0 at real overvoltage**:
  highest cell ≥ `OverVSetpoint` (default 4.20 V) – the signal for Victron to switch off all chargers.
  Released when the highest cell is back below 4.15 V.
- fixed: in TeslaBMSV2 the taper could wrap around to a huge CCL (~6500 A), because the current
  was stored as an unsigned number

**CVL** is controlled the way Victron systems expect:

- normal: CVL = `ChargeVsetpoint` × Scells (default 4.10 V × 6 = 24.6 V; storage mode: `StoreVsetpoint`)
- as soon as the highest cell rises above `ChargeVsetpoint`, the CVL is first set to at most battery
  voltage + 0.1 V (a CVL far above the battery voltage limits nothing) and then drops by at most
  **0.1 V per second**, towards *battery voltage − excess of that cell*, so the chargers hold the
  voltage neatly and that cell stays at the target
- when the highest cell is more than 10 mV below the target again, the CVL rises slowly (0.01 V/s)
- no fixed lower limit: with a badly balanced pack (e.g. five cells at 3.50 V and one at 4.15 V) the CVL
  goes as low as needed; only a safety net against a wrong measurement: `UnderVSetpoint` × Scells
- can be switched off (old behaviour: fixed CVL, CCL taper): menu `m` → `v`
- the status line in the serial monitor shows CVL, CCL and DCL

## Balancing

Passive (bleed resistors in the Tesla module), per string, per cell:

- only cells above **3.90 V** (menu `b` → `5`)
- **start** when a cell is more than **15 mV** above the lowest cell in its string (menu `b` → `l`)
- **stop** when it is only **5 mV** above it (menu `b` → `6`)
- strings cannot balance each other; incomplete strings are not balanced
- the number of balancing cells is now really a count of cells (previously the sum of bitmasks);
  in the pack details a `*` after the voltage shows which cells are being bled

## Detection and communication

- Each reply from a module is checked (address, register, length **and CRC**)
- The receive buffer is flushed before each command (old bytes no longer shift a reply)
- The original minimum wait times are kept: the module needs them to finish an ADC measurement
- `setupBoards()` verifies each newly assigned address and can no longer loop forever

## Serial menu

Open the menu with `s` in the serial monitor (115200 baud).

- `b` – Battery Settings (Scells, Pstrings, capacity **per string**, balancing)
- `m` – **Modules & Strings** (new): overview per string, `a` auto-detect, `f` search again, `r` renumber, `t` alarm delay, `v` CVL control, `s` CCL/DCL scaling
- `q` – back / save (EEPROM)

Settings from TeslaBMSV2 are taken over automatically (EEPROM migration); only the new fields get defaults.

## Other fixes compared to TeslaBMSV2

- ESS mode: delay timers for over/undervoltage no longer shared with the charge/discharge logic and no longer inverted
- Cell count per module is updated regardless of debug output
- "Series Cells Fault" only worked with debug output on; replaced by the string monitoring

Known and not yet fixed: some contactor/precharge timers (`mainconttimer`, `Pretimer`) are not
rollover-safe for `millis()` (after ~49.7 days of continuous operation).

## Licence

Based on TeslaBMSV2, Copyright (c) 2019 Simp ECO Engineering, MIT licence (see the header of
`TeslaBMS_Victron.ino`). The adaptations fall under the same licence.
