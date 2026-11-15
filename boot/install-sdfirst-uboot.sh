#!/bin/bash
# Boot the Orange Pi 5 from SD-card U-Boot that tries the SD card BEFORE USB.
# Run with:  sudo /home/viking/hunker/boot/install-sdfirst-uboot.sh      then reboot.
#
#  1. Checks: SPI backup exists; SD card holds the stock bootloader (idbloader at sector 64,
#     u-boot.itb at sector 16384) matching /usr/lib/u-boot. Aborts with NO changes otherwise.
#  2. Saves the SD bootloader area (sectors 0-32767) to /home/viking/sd-bootarea-backup-*.img
#  3. Writes u-boot-sdfirst.itb (see patch-uboot-bootorder.py) at sector 16384, reads it back.
#  4. Erases the SPI flash, so the boot ROM falls through to the SD card's bootloader.
#
# Undo SPI:  sudo flash_erase /dev/mtd0 0 0 && sudo dd if=/home/viking/spi-backup-20261114.img of=/dev/mtdblock0 conv=fsync
# Undo SD :  sudo dd if=/usr/lib/u-boot/u-boot.itb of=/dev/mmcblk1 bs=512 seek=16384 conv=notrunc,fsync
# If it won't boot at all: restore the SD card image on the laptop (see CLAUDE.md "Backups").
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
cd "$(dirname "$0")"

SD=/dev/mmcblk1
SPI_MTD=/dev/mtd0
SPI_BACKUP=/home/viking/spi-backup-20261114.img
STOCK_IDB=/usr/lib/u-boot/idbloader.img
STOCK_ITB=/usr/lib/u-boot/u-boot.itb
NEW_ITB=u-boot-sdfirst.itb

fail() { echo "ABORT: $*  (nothing has been changed)"; exit 1; }

# --- 1. checks (read-only) ---
[ "$(findmnt -no SOURCE /)" = "${SD}p1" ] || fail "root is not ${SD}p1"
[ "$(cat /sys/block/mmcblk1/mmcblk1p1/start)" = 32768 ] || fail "partition 1 does not start at sector 32768"
[ -f "$SPI_BACKUP" ] && [ "$(stat -c %s "$SPI_BACKUP")" = 16777216 ] || fail "SPI backup $SPI_BACKUP missing or wrong size"
[ -f "$NEW_ITB" ] || fail "$NEW_ITB not found in $(pwd)"
[ "$(stat -c %s "$NEW_ITB")" = "$(stat -c %s "$STOCK_ITB")" ] || fail "$NEW_ITB size differs from stock"
# (grep the file directly: "strings | grep -q" fails under pipefail when grep exits early)
grep -qaF 'boot_targets=mmc1 usb0 nvme0' "$NEW_ITB" || fail "$NEW_ITB does not have the SD-first boot order"
command -v flash_erase >/dev/null || fail "flash_erase not installed (mtd-utils)"

cmp -s -n "$(stat -c %s "$STOCK_IDB")" "$STOCK_IDB" <(dd if=$SD bs=512 skip=64 count=$(( ($(stat -c %s "$STOCK_IDB") + 511) / 512 )) status=none) \
    || fail "SD sector 64 does not hold the stock idbloader; SD would not be bootable without SPI"
cmp -s -n "$(stat -c %s "$STOCK_ITB")" "$STOCK_ITB" <(dd if=$SD bs=512 skip=16384 count=$(( ($(stat -c %s "$STOCK_ITB") + 511) / 512 )) status=none) \
    || fail "SD sector 16384 does not hold the stock u-boot.itb"
echo "checks OK: SD holds the stock bootloader; SPI backup present"

# --- 2. back up the SD bootloader area ---
BK=/home/viking/sd-bootarea-backup-$(date +%Y%m%d-%H%M%S).img
dd if=$SD of="$BK" bs=512 count=32768 status=none
chown viking:viking "$BK"
echo "saved SD sectors 0-32767 to $BK"

# --- 3. write the SD-first U-Boot and verify ---
dd if="$NEW_ITB" of=$SD bs=512 seek=16384 conv=notrunc,fsync status=none
sync
cmp -n "$(stat -c %s "$NEW_ITB")" "$NEW_ITB" <(dd if=$SD bs=512 skip=16384 count=$(( ($(stat -c %s "$NEW_ITB") + 511) / 512 )) iflag=direct status=none) \
    || { echo "ERROR: read-back of SD U-Boot does not match. SPI NOT erased, so the board still boots from SPI."; exit 1; }
echo "SD U-Boot written and verified (boot order: mmc1 usb0 ...)"

# --- 4. erase SPI so the boot ROM uses the SD bootloader ---
flash_erase $SPI_MTD 0 0
if [ "$(head -c 1048576 /dev/mtdblock0 | tr -d '\377' | wc -c)" != 0 ]; then
    echo "WARNING: first 1 MiB of SPI is not all 0xFF after erase"
fi
sync
echo
echo "DONE. SPI erased; the board will boot U-Boot from the SD card, SD first."
echo "Now: sudo reboot   (with the WiFi dongle plugged in, to test)"
