#!/usr/bin/env python3
"""
teensy_flash.py - flash TeslaBMS_Victron onto the Teensy from a Cerbo GX (Venus OS).
Supports Teensy 3.2 (SimpBMS V2) and Teensy 4.0.

Pure Python 3, no extra packages: uses /dev/hidraw (Linux) and the Teensy HalfKay
bootloader protocol (same as PJRC's teensy_loader_cli for the MK20DX256 / IMXRT1062).

  python3 teensy_flash.py --check            # show running and latest version (JSON)
  python3 teensy_flash.py --latest           # fetch the latest release from GitHub and flash (asks first)
  python3 teensy_flash.py --latest --yes     # same, without asking (used by Node-RED)
  python3 teensy_flash.py --file x.hex       # flash a local .hex
  add --board T32 / T40 to choose the Teensy model (default: auto = from the running
  firmware, otherwise Teensy 3.2); add --dry-run to do everything except writing

The model of the .hex file is checked against the chosen model (a Teensy 4 program
lives at 0x60000000 and starts with a FlexSPI boot header), so a wrong file is refused.

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

HALFKAY_VID, HALFKAY_PID = 0x16C0, 0x0478
CACHE_DIR = os.path.dirname(os.path.abspath(__file__))

# Teensy models. hex: release asset name of the firmware for this model.
BOARDS = {
    "T32": {"name": "Teensy 3.2", "code_size": 262144, "block": 1024, "base": 0x00000000,
            "hex": re.compile(r"^TeslaBMS_Victron-(v[0-9]+\.[0-9]+)\.hex$"), "erase_blocks": 1, "erase_timeout": 5.0},
    "T40": {"name": "Teensy 4.0", "code_size": 2031616, "block": 1024, "base": 0x60000000,
            "hex": re.compile(r"^TeslaBMS_Victron_T40-(v[0-9]+\.[0-9]+)\.hex$"), "erase_blocks": 5, "erase_timeout": 45.0},
}
DEFAULT_BOARD = "T32"   # firmware before v26.26 does not report its model: always a Teensy 3.2


def log(msg):
    print(msg, flush=True)


def _user():
    try:
        import pwd
        return pwd.getpwuid(os.getuid()).pw_name
    except Exception:
        return str(os.getuid())


# ---------------------------------------------------------------- versions
def read_running(timeout=12.0):
    """(version, model) from the JSON output of the Teensy (if JSON is on), e.g. ("v26.26", "T40")."""
    port = find_serial_port()
    if not port:
        return None, None
    try:
        fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None, None
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
                m = re.search(rb'\{"fw":"([0-9]+\.[0-9]+)"(?:,"hw":"([A-Z0-9]+)")?', buf)
                if m:
                    hw = m.group(2).decode() if m.group(2) else DEFAULT_BOARD
                    return "v" + m.group(1).decode(), hw
                buf = buf[-8192:]
            else:
                time.sleep(0.1)
    finally:
        os.close(fd)
    return None, None


def latest_release(board):
    """(version, hex url, sha256 url) of the firmware for this model in the latest release."""
    req = urllib.request.Request(API_LATEST, headers=UA)
    with urllib.request.urlopen(req, timeout=30) as r:
        rel = json.load(r)
    pat = BOARDS[board]["hex"]
    assets = rel.get("assets", [])
    for a in assets:
        m = pat.match(a["name"])
        if m:
            sha = [s for s in assets if s["name"] == a["name"] + ".sha256"]
            return m.group(1), a["browser_download_url"], (sha[0]["browser_download_url"] if sha else None)
    raise RuntimeError("latest release %s contains no firmware for the %s" % (rel.get("tag_name"), BOARDS[board]["name"]))


def download(url):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read()


# ---------------------------------------------------------------- hex file
def hex_board(text):
    """Model of a .hex file from its addresses: Teensy 4 programs live at 0x60000000."""
    base = 0
    for line in text.splitlines():
        line = line.strip()
        if len(line) >= 15 and line[7:9] == "04":
            base = int(line[9:13], 16) << 16
            if base >= 0x60000000:
                return "T40"
        elif len(line) >= 11 and line[7:9] == "00":
            return "T40" if base >= 0x60000000 else "T32"
    return None


def parse_ihex(text, board):
    b = BOARDS[board]
    code_size, block = b["code_size"], b["block"]
    image = bytearray(b"\xff" * code_size)
    used = bytearray(code_size // block)
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
            a = base + addr - b["base"]
            if a < 0 or a + length > code_size:
                raise ValueError("line %d: address 0x%X outside the %s flash" % (n, base + addr, b["name"]))
            image[a:a + length] = data
            for k in range(a // block, (a + length - 1) // block + 1):
                used[k] = 1
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
    # sanity check of the start of the program
    if board == "T32":
        # vector table: stack pointer in RAM, reset vector in flash
        sp = int.from_bytes(image[0:4], "little")
        rv = int.from_bytes(image[4:8], "little")
        if not (0x1FFF8000 <= sp <= 0x20008000) or not (0 < rv < code_size):
            raise ValueError("this does not look like a Teensy 3.2 program (SP=0x%08X, reset=0x%08X)" % (sp, rv))
    else:
        # FlexSPI NOR boot header "FCFB" at the start, image vector table (tag 0xD1) at 0x1000
        if bytes(image[0:4]) != b"FCFB" or image[0x1000] != 0xD1:
            raise ValueError("this does not look like a Teensy 4 program (no FlexSPI boot header)")
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


UDEV_RULE = "/etc/udev/rules.d/zz-teslabms-teensy.rules"


def bootloader_access_ok():
    """Can we open the HalfKay bootloader once the Teensy is in it? Root always can; another
    user (Node-RED runs as 'nodered') only with the udev rule from install-teensy-ignore.sh.
    Checked BEFORE the Teensy is sent to its bootloader, so a missing rule never leaves the
    BMS switched off."""
    if os.getuid() == 0:
        return True
    try:
        with open(UDEV_RULE) as f:
            return "0478" in f.read()
    except OSError:
        return False


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


def flash(image, used, board, dry_run=False):
    b = BOARDS[board]
    block = b["block"]
    blocks = [i for i in range(b["code_size"] // block) if i == 0 or used[i]]
    log("Program for the %s: %d blocks of %d bytes" % (b["name"], len(blocks), block))
    if dry_run:
        log("--dry-run: Teensy is NOT being flashed")
        return
    dev = find_halfkay()
    if not dev and not bootloader_access_ok():
        raise RuntimeError("no access to the Teensy bootloader for user '%s' (udev rule %s missing). "
                           "Run install-teensy-ignore.sh once via SSH as root, then try again. "
                           "Nothing was changed: the BMS keeps running." % (_user(), UDEV_RULE))
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
    # udev sets the access rights (MODE 0666) a moment AFTER the device appears:
    # opening it immediately can give "permission denied", so retry for a few seconds
    fd = None
    end = time.time() + 8
    while fd is None:
        try:
            fd = os.open(dev, os.O_RDWR)
        except PermissionError:
            if time.time() > end:
                break
            time.sleep(0.25)
    if fd is None:
        raise RuntimeError("no access to %s. Run install-teensy-ignore.sh once via SSH as root, then try again. "
                           "The Teensy is now in its bootloader (BMS off): run the update again (it continues "
                           "from the bootloader), or unplug/replug its USB to start the old firmware again." % dev)
    try:
        for k, blk in enumerate(blocks):
            addr = blk * block
            hdr = bytes([addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF]) + bytes(61)
            # the first block(s) erase the flash: on a Teensy 4 that takes several seconds
            hid_write(fd, hdr + bytes(image[addr:addr + block]),
                      b["erase_timeout"] if k < b["erase_blocks"] else 0.5)
            if k % 16 == 0 or k == len(blocks) - 1:
                log("  written %d/%d" % (k + 1, len(blocks)))
        log("Restarting Teensy")
        boot = bytes([0xFF, 0xFF, 0xFF]) + bytes(61 + block)
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
    ap.add_argument("--board", default="auto", choices=["auto"] + sorted(BOARDS),
                    help="Teensy model (auto = from the running firmware, otherwise %s)" % DEFAULT_BOARD)
    ap.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    ap.add_argument("--dry-run", action="store_true", help="everything except actually flashing")
    ap.add_argument("--no-serial", action="store_true",
                    help="do not read the serial port for the running version (Node-RED already reads it)")
    a = ap.parse_args()

    running, running_hw = (None, None) if a.no_serial else read_running(timeout=12.0 if a.check else 7.0)
    board = a.board if a.board != "auto" else (running_hw if running_hw in BOARDS else DEFAULT_BOARD)

    if a.check:
        out = {"running": running, "board": board, "latest": None, "error": None}
        try:
            out["latest"] = latest_release(board)[0]
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
        tag, url, sha_url = latest_release(board)
        log("Latest firmware for the %s: %s" % (BOARDS[board]["name"], tag))
        data = download(url)
        if sha_url:
            want = download(sha_url).decode().split()[0].lower()
            got = hashlib.sha256(data).hexdigest()
            if want != got:
                raise RuntimeError("SHA256 does not match - download corrupt, not flashed")
            log("SHA256 OK")
        try:
            with open(os.path.join(CACHE_DIR, "last-%s.hex" % board), "wb") as f:
                f.write(data)
        except OSError:
            pass  # cache only
        text = data.decode()
        label = tag

    hb = hex_board(text)
    if hb and hb != board:
        raise RuntimeError("this .hex is for the %s, but the Teensy is a %s - not flashed"
                           % (BOARDS[hb]["name"], BOARDS[board]["name"]))
    image, used = parse_ihex(text, board)
    log("Running now: %s   ->   new: %s  (%s)" % (running or "unknown", label, BOARDS[board]["name"]))
    if not a.yes and not a.dry_run:
        if input("Flash the Teensy? The BMS is off for a few seconds. [y/N] ").strip().lower() not in ("y", "yes"):
            log("Cancelled")
            return 1
    flash(image, used, board, a.dry_run)
    log("Done. The Teensy starts the new firmware (modules are counted again).")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("ERROR: %s" % e, flush=True)
        sys.exit(2)
