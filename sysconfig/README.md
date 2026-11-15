# System config outside the repo (copies, 2026-11-14)

Files placed on the Orange Pi 5 by hand during bring-up. Paths mirror the real ones
(`sysconfig/etc/...` → `/etc/...`). These are copies for reference/restore, not live files.

| File | Purpose |
|---|---|
| `etc/default/u-boot` | Device-tree overlays: I2C5-M3 + `hunker-i2c5-400k` (see `boot/`). Run `sudo u-boot-update` after editing. |
| `etc/systemd/network/80-can0.network` | `can0` at 1 Mbit/s at boot |
| `etc/modprobe.d/8851bu.conf` | Quiet the RTL8851BU WiFi driver's logging |
| `etc/udev/rules.d/60-gpio.rules` | `/dev/gpiochip*` → group `gpio` |
| `etc/security/limits.d/90-hunker-rtprio.conf` | Real-time priority for `viking` |
| `usr/src/rtl8851bu-1.0/dkms.conf` | Our DKMS config for the WiFi driver |

Not copied: the NetworkManager `Hotspot` profile (root-only, and it contains the WiFi password).
Other manual steps: `groupadd gpio`, `usermod -aG gpio,i2c viking`.
