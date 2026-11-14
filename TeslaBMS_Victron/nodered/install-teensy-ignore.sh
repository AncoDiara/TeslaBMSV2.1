#!/bin/sh
# TeslaBMS_Victron - run ONCE via SSH (as root) on the GX if the Node-RED flow reports "noperm".
# Makes VenusOS serial-starter ignore the Teensy (USB-ID 16c0:0483) and adds the rule to /data/rc.local,
# so it comes back automatically after a firmware update. Running it again changes nothing.
P=$(ls /dev/serial/by-id/ 2>/dev/null | grep -i -m1 teensy); if [ -n "$P" ]; then echo "PORT=/dev/serial/by-id/$P"; else echo "PORT="; fi
R=/etc/udev/rules.d/zz-teslabms-teensy.rules
L='ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", ENV{VE_SERVICE}="ignore"'
RW=/opt/victronenergy/swupdate-scripts/remount-rw.sh
if [ -f "$R" ]; then echo "UDEV=ok"; else [ -x "$RW" ] && "$RW" >/dev/null 2>&1; if echo "$L" > "$R" 2>/dev/null; then udevadm control --reload-rules; udevadm trigger --action=add --subsystem-match=tty; echo "UDEV=added"; else echo "UDEV=noperm"; fi; fi
if grep -q teslabms-teensy /data/rc.local 2>/dev/null; then echo "RCLOCAL=ok"; elif [ -w /data ]; then [ -f /data/rc.local ] || printf '#!/bin/sh\n' > /data/rc.local; printf '%s\n' "# teslabms-teensy: VE_SERVICE=ignore for the Teensy (comes back after a firmware update)" "[ -f $R ] || { [ -x $RW ] && $RW; echo '$L' > $R; udevadm control --reload-rules; udevadm trigger --action=add --subsystem-match=tty; }" >> /data/rc.local; chmod +x /data/rc.local; echo "RCLOCAL=added"; else echo "RCLOCAL=noperm"; fi
