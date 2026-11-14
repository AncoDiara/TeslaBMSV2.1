#!/usr/bin/env python3
"""
teensy_flash.py - flash TeslaBMS_Victron onto the Teensy 3.2 from a Cerbo GX (Venus OS).

Pure Python 3, no extra packages: uses /dev/hidraw (Linux) and the Teensy HalfKay
bootloader protocol (same as PJRC's teensy_loader_cli for the MK20DX256).

  python3 teensy_flash.py --check            # show running and latest version (JSON)
  python3 teensy_flash.py --latest           # fetch the latest release from GitHub and flash (asks first)
  python3 teensy_flash.py --latest --yes     # same, without asking (used by Node-RED)
  python3 teensy_flash.py --file x.hex       # flash a local .hex
  add --dry-run to do everything except writing to the Teensy

Safe failure: if flashing is interrupted, the Teensy stays in its bootloader and the BMS
is off (Victron stops charging/discharging after 3 s). Just run it again, or press the
button on the Teensy and use the Teensy Loader on a PC.
"""
import argparse
import glob
import hashlib
import json
import os
import re
import sys
import termios
import time
import urllib.request

REPO = "AncoDiara/TeslaBMSV2.1"
API_LATEST = "https://api.github.com/repos/%s/releases/latest" % REPO
UA = {"User-Agent": "teslabms-flasher"}

# Teensy 3.2 (MK20DX256)
CODE_SIZE = 262144
BLOCK_SIZE = 1024
HALFKAY_VID, HALFKAY_PID = 0x16C0, 0x0478
CACHE_DIR = "/data/teslabms"


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------- versions
def read_running_version(timeout=12.0):
    """Read the 'fw' field from the JSON output of the Teensy (if JSON is on)."""
    port = find_serial_port()
    if not port:
        return None
    try:
        fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None
    buf = b""
    end = time.time() + timeout
    try:
        while time.time() < end:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                chunk = b""
            if chunk:
                buf += chunk
                m = re.search(rb'"fw":"([0-9]+\.[0-9]+)"', buf)
                if m:
                    return "v" + m.group(1).decode()
                buf = buf[-8192:]
            else:
                time.sleep(0.1)
    finally:
        os.close(fd)
    return None


def latest_release():
    req = urllib.request.Request(API_LATEST, headers=UA)
    with urllib.request.urlopen(req, timeout=30) as r:
        rel = json.load(r)
    hexa = [a for a in rel.get("assets", []) if a["name"].endswith(".hex")]
    sha = [a for a in rel.get("assets", []) if a["name"].endswith(".hex.sha256")]
    if not hexa:
        raise RuntimeError("latest release %s contains no .hex" % rel.get("tag_name"))
    return rel["tag_name"], hexa[0]["browser_download_url"], (sha[0]["browser_download_url"] if sha else None)


def download(url):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read()


# ---------------------------------------------------------------- hex file
def parse_ihex(text):
    image = bytearray(b"\xff" * CODE_SIZE)
    used = bytearray(CODE_SIZE // BLOCK_SIZE)
    base = 0
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError("line %d: not Intel HEX" % n)
        raw = bytes.fromhex(line[1:])
        if (sum(raw) & 0xFF) != 0:
            raise ValueError("line %d: checksum error" % n)
        length, addr, rtype, data = raw[0], (raw[1] << 8) | raw[2], raw[3], raw[4:4 + raw[0]]
        if rtype == 0:
            a = base + addr
            if a + length > CODE_SIZE:
                raise ValueError("line %d: address 0x%X outside Teensy 3.2 flash" % (n, a))
            image[a:a + length] = data
            for b in range(a // BLOCK_SIZE, (a + length - 1) // BLOCK_SIZE + 1):
                used[b] = 1
        elif rtype == 1:
            break
        elif rtype == 2:
            base = ((data[0] << 8) | data[1]) << 4
        elif rtype == 4:
            base = ((data[0] << 8) | data[1]) << 16
        elif rtype in (3, 5):
            pass
        else:
            raise ValueError("line %d: unknown record type %d" % (n, rtype))
    # sanity check: vector table of a Teensy 3.2 program (stack pointer in RAM, reset vector in flash)
    sp = int.from_bytes(image[0:4], "little")
    rv = int.from_bytes(image[4:8], "little")
    if not (0x1FFF8000 <= sp <= 0x20008000) or not (0 < rv < CODE_SIZE):
        raise ValueError("this does not look like a Teensy 3.2 program (SP=0x%08X, reset=0x%08X)" % (sp, rv))
    return image, used


# ---------------------------------------------------------------- Teensy
def find_serial_port():
    p = sorted(glob.glob("/dev/serial/by-id/*eensy*"))
    if p:
        return p[0]
    p = sorted(glob.glob("/dev/ttyACM*"))
    return p[0] if p else None


def find_halfkay():
    for d in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        try:
            with open(os.path.join(d, "device", "uevent")) as f:
                ue = f.read().upper()
        except OSError:
            continue
        if "HID_ID=0003:%08X:%08X" % (HALFKAY_VID, HALFKAY_PID) in ue:
            return "/dev/" + os.path.basename(d)
    return None


def soft_reboot(port):
    """Opening the Teensy serial port at 134 baud makes Teensyduino jump to the bootloader."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attr = termios.tcgetattr(fd)
        attr[4] = termios.B134
        attr[5] = termios.B134
        termios.tcsetattr(fd, termios.TCSANOW, attr)
    finally:
        os.close(fd)


def hid_write(fd, payload, timeout):
    """Write one HalfKay report (report ID 0 + payload); retry while the Teensy is busy."""
    end = time.time() + timeout
    while True:
        try:
            if os.write(fd, b"\x00" + payload) > 0:
                return
        except OSError:
            pass
        if time.time() > end:
            raise TimeoutError("Teensy is not responding")
        time.sleep(0.01)


def flash(image, used, dry_run=False):
    blocks = [i for i in range(CODE_SIZE // BLOCK_SIZE) if i == 0 or used[i]]
    log("Program: %d blocks of %d bytes" % (len(blocks), BLOCK_SIZE))
    if dry_run:
        log("--dry-run: Teensy is NOT being flashed")
        return
    dev = find_halfkay()
    if not dev:
        port = find_serial_port()
        if not port:
            raise RuntimeError("Teensy not found (no serial port and no bootloader)")
        log("Teensy to bootloader via %s" % port)
        soft_reboot(port)
        end = time.time() + 15
        while not dev and time.time() < end:
            time.sleep(0.25)
            dev = find_halfkay()
        if not dev:
            raise RuntimeError("bootloader did not appear; press the button on the Teensy and run again")
    log("Bootloader found: %s" % dev)
    fd = os.open(dev, os.O_RDWR)
    try:
        for k, b in enumerate(blocks):
            addr = b * BLOCK_SIZE
            hdr = bytes([addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF]) + bytes(61)
            hid_write(fd, hdr + bytes(image[addr:addr + BLOCK_SIZE]), 5.0 if k == 0 else 0.5)
            if k % 16 == 0 or k == len(blocks) - 1:
                log("  written %d/%d" % (k + 1, len(blocks)))
        log("Restarting Teensy")
        boot = bytes([0xFF, 0xFF, 0xFF]) + bytes(61 + BLOCK_SIZE)
        try:
            hid_write(fd, boot, 0.5)
        except TimeoutError:
            pass  # the Teensy restarts immediately; no answer is normal
    finally:
        os.close(fd)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Flash TeslaBMS_Victron onto the Teensy from the Cerbo GX")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true", help="show running and latest version (JSON)")
    g.add_argument("--latest", action="store_true", help="fetch the latest release from GitHub and flash")
    g.add_argument("--file", help="flash a local .hex file")
    ap.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    ap.add_argument("--dry-run", action="store_true", help="everything except actually flashing")
    ap.add_argument("--no-serial", action="store_true",
                    help="do not read the serial port for the running version (Node-RED already reads it)")
    a = ap.parse_args()

    if a.check:
        out = {"running": None if a.no_serial else read_running_version(), "latest": None, "error": None}
        try:
            out["latest"] = latest_release()[0]
        except Exception as e:
            out["error"] = str(e)
        out["update"] = bool(out["latest"] and out["running"] and out["latest"] != out["running"])
        print(json.dumps(out))
        return 0

    if a.file:
        with open(a.file) as f:
            text = f.read()
        label = os.path.basename(a.file)
    else:
        tag, url, sha_url = latest_release()
        log("Latest release: %s" % tag)
        data = download(url)
        if sha_url:
            want = download(sha_url).decode().split()[0].lower()
            got = hashlib.sha256(data).hexdigest()
            if want != got:
                raise RuntimeError("SHA256 does not match - download corrupt, not flashed")
            log("SHA256 OK")
        os.makedirs(CACHE_DIR, exist_ok=True)
        with open(os.path.join(CACHE_DIR, "last.hex"), "wb") as f:
            f.write(data)
        text = data.decode()
        label = tag

    image, used = parse_ihex(text)
    running = None if a.no_serial else read_running_version(timeout=7.0)
    log("Running now: %s   ->   new: %s" % (running or "unknown", label))
    if not a.yes and not a.dry_run:
        if input("Flash the Teensy? The BMS is off for a few seconds. [y/N] ").strip().lower() not in ("y", "yes"):
            log("Cancelled")
            return 1
    flash(image, used, a.dry_run)
    log("Done. The Teensy starts the new firmware (modules are counted again).")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("ERROR: %s" % e, flush=True)
        sys.exit(2)
