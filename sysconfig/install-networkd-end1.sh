#!/bin/bash
# Move the Pi's Ethernet (end1) from NetworkManager to systemd-networkd.
#
# Run it detached, so it can finish even if your SSH session drops:
#   sudo systemd-run --unit=hunker-net-switch /home/viking/hunker/sysconfig/install-networkd-end1.sh
#   journalctl -u hunker-net-switch      # see what it did
#
# Undo:
#   sudo rm /etc/systemd/network/20-end1.network /etc/NetworkManager/conf.d/90-hunker-unmanaged.conf
#   sudo networkctl reload && sudo systemctl reload NetworkManager
set -euo pipefail
SRC="$(cd "$(dirname "$0")" && pwd)"

install -m 644 "$SRC/etc/systemd/network/20-end1.network"            /etc/systemd/network/
install -m 644 "$SRC/etc/NetworkManager/conf.d/90-hunker-unmanaged.conf" /etc/NetworkManager/conf.d/

systemctl reload NetworkManager      # NM drops end1
sleep 2
networkctl reload
networkctl reconfigure end1          # networkd takes it: DHCP + fixed address
sleep 8
networkctl status end1 --no-pager || true
ip -4 addr show end1
ip route
