#!/usr/bin/env python3
"""
solaredge_ctl.py - control a SolarEdge StorEdge inverter (e.g. SE5K-SE10K-RWS) over Modbus TCP.
Pure Python 3, no extra packages (used by the Node-RED flow on the Cerbo GX).

Enable Modbus TCP on the inverter first: SetApp -> Site Communication -> Modbus TCP -> Enable
(port 1502, device ID 1).

  python3 solaredge_ctl.py --host 192.168.1.50 status
  python3 solaredge_ctl.py --host 192.168.1.50 set --mode 4 --discharge 2000 --timeout 60
  python3 solaredge_ctl.py --host 192.168.1.50 set --mode 2 --charge 5000 --timeout 60 --default 2

Every call prints one JSON line (also on errors: {"error": "..."}).

StorEdge control block (holding registers, 0-based; float32/uint32 with the low word first):
  57348 storage control mode   0 disabled, 1 max. self consumption, 2 time of use, 3 backup, 4 REMOTE CONTROL
  57349 AC charge policy       0 disabled, 1 always allowed, 2 fixed energy limit, 3 percent of production
  57354 default mode           mode after the command timeout (values as command mode)
  57355 command timeout (s)    uint32
  57357 command mode           0 solar only (battery off), 1 charge from clipped solar, 2 charge from solar,
                               3 charge from solar and grid, 4 discharge to maximize export,
                               5 discharge to minimize import (needs a SolarEdge meter), 7 max. self consumption
  57358 charge limit (W)       float32
  57360 discharge limit (W)    float32
Battery 1 (57600 +): 57712 DC voltage, 57714 DC current, 57716 DC power (W), 57730 SOH, 57732 SOE (%),
  57734 status (0 off, 1 standby, 2 init, 3 charge, 4 discharge, 5 fault, 6 preserve charge, 7 idle, 10 power saving)
Inverter (SunSpec): 40083 AC power + 40084 scale factor, 40107 status.
The command, timeout and limits are not stored by the inverter: send them again before the timeout runs out.
"""
import argparse
import json
import socket
import struct
import sys

REG_CONTROL_MODE = 57348
REG_AC_POLICY = 57349
REG_DEFAULT_MODE = 57354
REG_TIMEOUT = 57355
REG_COMMAND_MODE = 57357
REG_CHARGE_LIMIT = 57358
REG_DISCHARGE_LIMIT = 57360
REG_BATTERY = 57708          # 57708 .. 57737
REG_INV_AC = 40083           # AC power, scale factor at 40084
REG_INV_STATUS = 40107

BATTERY_STATUS = {0: "off", 1: "standby", 2: "init", 3: "charge", 4: "discharge", 5: "fault",
                  6: "preserve charge", 7: "idle", 10: "power saving"}
INV_STATUS = {1: "off", 2: "sleeping", 3: "starting", 4: "producing", 5: "throttled", 6: "shutting down",
              7: "fault", 8: "standby"}


class Modbus:
    def __init__(self, host, port, unit, timeout=3.0):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.settimeout(timeout)
        self.unit = unit
        self.tid = 0

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass

    def _recv(self, n):
        b = b""
        while len(b) < n:
            c = self.s.recv(n - len(b))
            if not c:
                raise IOError("connection closed by the inverter")
            b += c
        return b

    def _call(self, pdu):
        self.tid = (self.tid + 1) & 0xFFFF
        self.s.sendall(struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, self.unit) + pdu)
        hdr = self._recv(7)
        tid, _, length, _ = struct.unpack(">HHHB", hdr)
        body = self._recv(length - 1)
        if body[0] & 0x80:
            raise IOError("Modbus exception %d (function 0x%02X)" % (body[1], body[0] & 0x7F))
        return body

    def read(self, addr, count):
        body = self._call(struct.pack(">BHH", 3, addr, count))
        return list(struct.unpack(">%dH" % (body[1] // 2), body[2:2 + body[1]]))

    def write(self, addr, regs):
        self._call(struct.pack(">BHHB", 16, addr, len(regs), 2 * len(regs)) + struct.pack(">%dH" % len(regs), *regs))


def f32(regs, i):            # float32, low word first
    return struct.unpack(">f", struct.pack(">HH", regs[i + 1], regs[i]))[0]


def u32(regs, i):            # uint32, low word first
    return (regs[i + 1] << 16) | regs[i]


def f32_regs(v):
    hi, lo = struct.unpack(">HH", struct.pack(">f", float(v)))
    return [lo, hi]


def u32_regs(v):
    v = int(v)
    return [v & 0xFFFF, (v >> 16) & 0xFFFF]


def status(m):
    out = {}
    c = m.read(REG_CONTROL_MODE, 14)          # 57348 .. 57361
    out["control_mode"] = c[0]
    out["ac_policy"] = c[1]
    out["default_mode"] = c[6]
    out["timeout"] = u32(c, 7)
    out["command_mode"] = c[9]
    out["charge_limit"] = round(f32(c, 10))
    out["discharge_limit"] = round(f32(c, 12))
    try:
        b = m.read(REG_BATTERY, 30)           # 57708 .. 57737
        out["bat_temp"] = round(f32(b, 0), 1)
        out["bat_v"] = round(f32(b, 4), 2)
        out["bat_i"] = round(f32(b, 6), 1)
        out["bat_p"] = round(f32(b, 8))
        out["bat_soh"] = round(f32(b, 22), 1)
        out["bat_soe"] = round(f32(b, 24), 1)
        st = u32(b, 26)
        out["bat_status"] = st
        out["bat_status_text"] = BATTERY_STATUS.get(st, "status %d" % st)
    except IOError as e:
        out["bat_error"] = str(e)
    try:
        a = m.read(REG_INV_AC, 2)
        sf = struct.unpack(">h", struct.pack(">H", a[1]))[0]
        p = struct.unpack(">h", struct.pack(">H", a[0]))[0]
        out["inv_ac_p"] = round(p * (10 ** sf))
        st = m.read(REG_INV_STATUS, 1)[0]
        out["inv_status"] = st
        out["inv_status_text"] = INV_STATUS.get(st, "status %d" % st)
    except IOError as e:
        out["inv_error"] = str(e)
    return out


def main():
    ap = argparse.ArgumentParser(description="SolarEdge StorEdge control over Modbus TCP")
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=1502)
    ap.add_argument("--unit", type=int, default=1)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    s = sub.add_parser("set")
    s.add_argument("--mode", type=int, required=True, choices=[0, 1, 2, 3, 4, 5, 7])
    s.add_argument("--charge", type=float, help="charge limit in W")
    s.add_argument("--discharge", type=float, help="discharge limit in W")
    s.add_argument("--timeout", type=int, default=60, help="s; after this the inverter uses the default mode")
    s.add_argument("--default", type=int, choices=[0, 1, 2, 3, 4, 5, 7], help="default mode after the timeout")
    s.add_argument("--ac-policy", type=int, choices=[0, 1, 2, 3], help="AC charge policy (needed for mode 3)")
    a = ap.parse_args()

    m = Modbus(a.host, a.port, a.unit)
    try:
        if a.cmd == "set":
            cur = m.read(REG_CONTROL_MODE, 1)[0]
            if cur != 4:
                m.write(REG_CONTROL_MODE, [4])                 # remote control
            if a.ac_policy is not None:
                m.write(REG_AC_POLICY, [a.ac_policy])
            if a.default is not None:
                m.write(REG_DEFAULT_MODE, [a.default])
            m.write(REG_TIMEOUT, u32_regs(max(10, min(86400, a.timeout))))
            if a.charge is not None:
                m.write(REG_CHARGE_LIMIT, f32_regs(max(0.0, a.charge)))
            if a.discharge is not None:
                m.write(REG_DISCHARGE_LIMIT, f32_regs(max(0.0, a.discharge)))
            m.write(REG_COMMAND_MODE, [a.mode])
        out = status(m)
        out["ok"] = True
        print(json.dumps(out))
        return 0
    finally:
        m.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception as e:
        print(json.dumps({"ok": False, "error": str(e)}))
        sys.exit(2)
