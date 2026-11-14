#!/usr/bin/env python3
"""
flow_update.py - update the "Tesla BMS" Node-RED dashboard flow on a Cerbo GX (Venus OS Large).

Replaces ONLY the "Tesla BMS" tab and its own config nodes in the Node-RED flows file with the
version from the latest GitHub release, makes a backup first, and restarts Node-RED.
Other flows are left untouched. An existing Dashboard 2.0 base (ui-base) is never removed:
if one exists, the Tesla BMS page is attached to it.

  python3 flow_update.py --check                 # installed and latest flow version (JSON)
  python3 flow_update.py --latest [--yes]        # update to the latest release
  python3 flow_update.py --file flow.json [--yes]
  options: --flows <path to flows.json>, --no-restart, --dry-run
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.request

REPO = "AncoDiara/TeslaBMSV2.1"
ASSET = "tesla-bms-dashboard.json"
API_LATEST = "https://api.github.com/repos/%s/releases/latest" % REPO
UA = {"User-Agent": "teslabms-flow-updater"}
TAB_LABEL = "Tesla BMS"
VERSION_ENV = "TESLABMS_FLOW_VERSION"


def log(msg):
    print(msg, flush=True)


def find_flows_file(explicit=None):
    if explicit:
        return explicit
    cands = []
    for pat in ("/data/home/nodered/.node-red/flows*.json",
                "/data/home/*/.node-red/flows*.json",
                os.path.expanduser("~/.node-red/flows*.json")):
        cands += [p for p in glob.glob(pat) if "_cred" not in p and not p.endswith(".backup")]
    cands = sorted(set(cands), key=lambda p: os.path.getmtime(p), reverse=True)
    if not cands:
        raise RuntimeError("Node-RED flows file not found (pass --flows <path>)")
    return cands[0]


def latest_release():
    req = urllib.request.Request(API_LATEST, headers=UA)
    with urllib.request.urlopen(req, timeout=30) as r:
        rel = json.load(r)
    for a in rel.get("assets", []):
        if a["name"] == ASSET:
            return rel["tag_name"], a["browser_download_url"]
    raise RuntimeError("latest release %s contains no %s" % (rel.get("tag_name"), ASSET))


def download(url):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read()


def flow_version(nodes):
    for n in nodes:
        if n.get("type") == "tab" and n.get("label") == TAB_LABEL:
            for e in n.get("env", []):
                if e.get("name") == VERSION_ENV:
                    return e.get("value")
            return "unknown (old version without number)"
    return None


def merge(current, new):
    """Return the new list of nodes: current minus the old Tesla BMS parts, plus the new flow."""
    new_ids = {n["id"] for n in new}
    # old tabs: same id as the new tab, or with label "Tesla BMS"
    old_tabs = {n["id"] for n in current if n.get("type") == "tab" and (n["id"] in new_ids or n.get("label") == TAB_LABEL)}
    old_pages = {n["id"] for n in current if n.get("type") == "ui-page" and (n["id"] in new_ids or n.get("path") == "/tesla-bms")}
    existing_bases = [n for n in current if n.get("type") == "ui-base" and n["id"] not in new_ids]
    # which base did the old page use (the user may have pointed it at their own base)?
    old_page_base = None
    for n in current:
        if n["id"] in old_pages and n.get("ui"):
            old_page_base = n["ui"]

    def is_old(n):
        if n.get("type") == "ui-base":
            return False                      # never remove a dashboard base
        if n["id"] in new_ids or n["id"] in old_tabs or n.get("z") in old_tabs:
            return True
        if n.get("type") == "ui-page" and n["id"] in old_pages:
            return True
        if n.get("type") == "ui-group" and n.get("page") in old_pages:
            return True
        if n.get("type") == "serial-port" and n.get("name") == "Teensy USB":
            return True
        if n.get("type") == "ui-theme" and n.get("name") == "Tesla BMS theme":
            return True
        return False

    kept = [n for n in current if not is_old(n)]
    removed = len(current) - len(kept)
    kept_ids = {n["id"] for n in kept}

    incoming = []
    new_base_ids = {n["id"] for n in new if n.get("type") == "ui-base"}
    use_base = None
    if old_page_base and old_page_base in kept_ids:
        use_base = old_page_base
    elif existing_bases:
        use_base = existing_bases[0]["id"]
    for n in new:
        n = dict(n)
        if n.get("type") == "ui-base":
            if use_base or n["id"] in kept_ids:
                continue                      # existing base is used / already present
        if n.get("type") == "ui-page" and use_base:
            n["ui"] = use_base
        incoming.append(n)
    return kept + incoming, removed, len(incoming), use_base, new_base_ids


def _cmdline(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            return [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
    except OSError:
        return []


def _ppid(pid):
    try:
        with open("/proc/%d/stat" % pid) as f:
            return int(f.read().rsplit(")", 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        return 0


def find_nodered_pid():
    """The Node-RED process: first look up our own parent chain (we run from an exec node),
    then any process of our own user with node-red in its command line."""
    def is_red(argv):
        # node .../node-red/red.js  or  node-red ...   (not a shell or python that merely mentions it)
        if not argv or os.path.basename(argv[0]) in ("sh", "bash", "python3", "python"):
            return False
        return any(os.path.basename(a) in ("node-red", "red.js") for a in argv[:3])
    pid = os.getppid()
    while pid > 1:
        if is_red(_cmdline(pid)):
            return pid
        pid = _ppid(pid)
    me = os.getuid()
    for d in os.listdir("/proc"):
        if d.isdigit() and int(d) != os.getpid():
            try:
                if os.stat("/proc/" + d).st_uid != me:
                    continue
            except OSError:
                continue
            c = _cmdline(int(d))
            if is_red(c):
                return int(d)
    return None


def restart_nodered():
    """Restart Node-RED. As root via svc; as user nodered (Venus OS runs Node-RED without root)
    svc is not allowed, so stop our own Node-RED process: the service supervisor starts it again."""
    for svc in ("/service/node-red-venus", "/service/node-red"):
        if os.path.isdir(svc):
            r = subprocess.run(["svc", "-t", svc], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            out = r.stdout.decode(errors="replace")
            if r.returncode == 0 and "unable" not in out and "denied" not in out:
                return "svc -t %s" % svc
            break
    pid = find_nodered_pid()
    if pid:
        # delayed and detached, so this output still reaches the dashboard
        subprocess.Popen(["sh", "-c", "sleep 2; kill %d" % pid], start_new_session=True,
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return "process %d is stopped, the supervisor starts it again" % pid
    return None


def main():
    ap = argparse.ArgumentParser(description="Update the Tesla BMS Node-RED flow")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--latest", action="store_true")
    g.add_argument("--file")
    ap.add_argument("--flows", help="path to the Node-RED flows file")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-restart", action="store_true")
    a = ap.parse_args()

    flows_path = find_flows_file(a.flows)
    with open(flows_path) as f:
        current = json.load(f)
    if isinstance(current, dict):             # old format {"flows":[...], "rev":...}
        current = current.get("flows", [])
    installed = flow_version(current)

    if a.check:
        out = {"installed": installed, "latest": None, "error": None, "flows": flows_path}
        try:
            out["latest"] = latest_release()[0]
        except Exception as e:
            out["error"] = str(e)
        out["update"] = bool(out["latest"] and out["latest"] != installed)
        print(json.dumps(out))
        return 0

    if a.file:
        with open(a.file) as f:
            new = json.load(f)
        label = os.path.basename(a.file)
    else:
        tag, url = latest_release()
        new = json.loads(download(url).decode())
        label = tag
    if not isinstance(new, list) or not any(n.get("type") == "tab" and n.get("label") == TAB_LABEL for n in new):
        raise RuntimeError("this is not a Tesla BMS flow")

    log("Flows file: %s" % flows_path)
    log("Installed: %s   ->   new: %s" % (installed or "not installed", flow_version(new) or label))
    merged, removed, added, base, _ = merge(current, new)
    log("Replacing: %d old nodes removed, %d new nodes%s" % (
        removed, added, (" (page attached to existing dashboard base %s)" % base) if base else ""))
    if a.dry_run:
        log("--dry-run: nothing written")
        return 0
    if not a.yes:
        if input("Update the flow and restart Node-RED? [y/N] ").strip().lower() not in ("y", "yes"):
            log("Cancelled")
            return 1
    backup = "%s.teslabms-backup-%s" % (flows_path, time.strftime("%Y%m%d-%H%M%S"))
    shutil.copy2(flows_path, backup)
    log("Backup: %s" % backup)
    tmp = flows_path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(merged, f, indent=4)
    st = os.stat(flows_path)
    if os.getuid() == 0:                      # run as root via SSH: keep Node-RED (user nodered) the owner
        os.chown(tmp, st.st_uid, st.st_gid)
        os.chown(backup, st.st_uid, st.st_gid)
    os.chmod(tmp, st.st_mode & 0o777)
    os.replace(tmp, flows_path)
    log("Flow written.")
    if a.no_restart:
        log("Restart Node-RED yourself to activate the new version.")
    else:
        svc = restart_nodered()
        log(("Node-RED is restarting (%s)." % svc) if svc else
            "Could not restart Node-RED: restart it yourself (Venus: Settings -> Integrations -> Node-RED off/on).")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("ERROR: %s" % e, flush=True)
        sys.exit(2)
