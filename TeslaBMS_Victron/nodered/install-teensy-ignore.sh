#!/bin/sh
# TeslaBMS_Victron - run ONCE via SSH (as root) on the GX. Running it again changes nothing.
# - makes VenusOS serial-starter ignore the Teensy (USB-ID 16c0:0483)
# - gives Node-RED (user nodered) access to the Teensy bootloader (16c0:0478) for firmware updates
# - both rules are added to /data/rc.local, so they come back after a Venus OS update
# - makes /data/teslabms writable for Node-RED (update scripts)
P=$(ls /dev/serial/by-id/ 2>/dev/null | grep -i -m1 teensy); if [ -n "$P" ]; then echo "PORT=/dev/serial/by-id/$P"; else echo "PORT="; fi
R=/etc/udev/rules.d/zz-teslabms-teensy.rules
RW=/opt/victronenergy/swupdate-scripts/remount-rw.sh
if grep -q 0478 "$R" 2>/dev/null; then echo "UDEV=ok"; else [ -x "$RW" ] && "$RW" >/dev/null 2>&1; if printf '%s\n' 'ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ENV{VE_SERVICE}="ignore"' 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0478", MODE="0666"' > "$R" 2>/dev/null; then udevadm control --reload-rules; udevadm trigger --action=add --subsystem-match=tty; echo "UDEV=added"; else echo "UDEV=noperm"; fi; fi
if grep -q teslabms-teensy-v2 /data/rc.local 2>/dev/null; then echo "RCLOCAL=ok"; elif [ -w /data/rc.local ] || { [ ! -e /data/rc.local ] && [ -w /data ]; }; then [ -f /data/rc.local ] || printf '#!/bin/sh\n' > /data/rc.local; sed -i '/teslabms-teensy/d' /data/rc.local; printf '%s\n' "# teslabms-teensy-v2: VE_SERVICE=ignore + bootloader access for the Teensy (comes back after a firmware update)" "grep -q 0478 $R 2>/dev/null || { [ -x $RW ] && $RW; printf '%s\\n' 'ACTION==\"add\", SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"16c0\", ATTRS{idProduct}==\"0483\", ENV{VE_SERVICE}=\"ignore\"' 'SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"16c0\", ATTRS{idProduct}==\"0478\", MODE=\"0666\"' > $R; udevadm control --reload-rules; udevadm trigger --action=add --subsystem-match=tty; } # teslabms-teensy" >> /data/rc.local; chmod +x /data/rc.local; echo "RCLOCAL=added"; else echo "RCLOCAL=noperm"; fi
mkdir -p /data/teslabms
if id nodered >/dev/null 2>&1; then chown -R nodered /data/teslabms && echo "SCRIPTDIR=ok (owner nodered)"; else echo "SCRIPTDIR=ok"; fi
