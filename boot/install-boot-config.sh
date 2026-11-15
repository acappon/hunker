#!/bin/bash
# Hunker boot/system config. Run with:  sudo ./install-boot-config.sh   then reboot.
# Safe to re-run. Re-run after a kernel package upgrade (new /lib/firmware/<kernel>/ dir).
#  1. Installs hunker-i2c5-400k.dtbo (I2C5 at 400 kHz for the BNO085).
#  2. Sets U_BOOT_FDT_OVERLAYS (paths RELATIVE to /lib/firmware/<kernel>/ — absolute
#     paths are silently ignored by u-boot-update) and regenerates extlinux.conf.
#  3. Adds usb-storage.quirks=0bda:1a2b:i to /etc/kernel/cmdline (WiFi dongle fake CD-ROM).
#  4. Lets user viking use real-time thread priority (IMU interrupt thread, 200 Hz loop).
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
cd "$(dirname "$0")"

K=$(uname -r)
OVL_DIR=/lib/firmware/$K/device-tree/rockchip/overlay
[ -f "$OVL_DIR/rk3588-i2c5-m3.dtbo" ] || { echo "ERROR: $OVL_DIR/rk3588-i2c5-m3.dtbo missing"; exit 1; }

install -m 0644 hunker-i2c5-400k.dtbo "$OVL_DIR/"
echo "installed $OVL_DIR/hunker-i2c5-400k.dtbo"

# Kernel command line (read by u-boot-update via /usr/share/u-boot-menu/conf.d/ubuntu.conf).
# usb-storage.quirks=0bda:1a2b:i — ignore the WiFi dongle's fake CD-ROM (Realtek ZeroCD).
# Without it the built-in usb-storage/sr driver retries that "CD" for ~6 minutes during
# early boot, before the root filesystem is mounted. usb_modeswitch still switches the
# dongle (it uses libusb, not usb-storage).
QUIRK="usb-storage.quirks=0bda:1a2b:i"
cp -n /etc/kernel/cmdline /etc/kernel/cmdline.orig || true
if ! grep -qF "$QUIRK" /etc/kernel/cmdline; then
    printf '%s %s' "$(cat /etc/kernel/cmdline)" "$QUIRK" > /etc/kernel/cmdline.new
    mv /etc/kernel/cmdline.new /etc/kernel/cmdline
fi
echo "kernel cmdline: $(cat /etc/kernel/cmdline)"

OVERLAYS="device-tree/rockchip/overlay/rk3588-i2c5-m3.dtbo device-tree/rockchip/overlay/hunker-i2c5-400k.dtbo"
cp -n /etc/default/u-boot /etc/default/u-boot.orig || true
if grep -q '^U_BOOT_FDT_OVERLAYS=' /etc/default/u-boot; then
    sed -i "s|^U_BOOT_FDT_OVERLAYS=.*|U_BOOT_FDT_OVERLAYS=\"$OVERLAYS\"|" /etc/default/u-boot
else
    echo "U_BOOT_FDT_OVERLAYS=\"$OVERLAYS\"" >> /etc/default/u-boot
fi
u-boot-update
echo "extlinux.conf:"
grep fdtoverlays /boot/extlinux/extlinux.conf || { echo "ERROR: no fdtoverlays line in extlinux.conf"; exit 1; }
[ "$(grep -c 'fdtoverlays.*hunker-i2c5-400k' /boot/extlinux/extlinux.conf)" -ge 1 ] || { echo "ERROR: 400k overlay not listed"; exit 1; }
grep -q "append .*$QUIRK" /boot/extlinux/extlinux.conf || { echo "ERROR: usb-storage quirk not on the kernel command line in extlinux.conf"; exit 1; }
echo "extlinux.conf append lines include $QUIRK"

echo 'viking  -  rtprio  90' > /etc/security/limits.d/90-hunker-rtprio.conf
echo "wrote /etc/security/limits.d/90-hunker-rtprio.conf"

echo "OK. Now: sudo reboot"
