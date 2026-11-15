#!/bin/bash
# Put the bootloader back in SPI flash: stock rkspi_loader.img with the SD-first U-Boot
# (u-boot-sdfirst.itb, see patch-uboot-bootorder.py) at 0x800000.
# Run with:  sudo /home/viking/hunker/boot/install-spi-sdfirst.sh     then test with: sudo reboot
#
# Why: with SPI erased (install-sdfirst-uboot.sh, 2026-11-14) the boot ROM must read the
# bootloader from the SD card. After a WARM reboot the card is still in UHS SDR104 (1.8 V)
# mode, the ROM can't read it, and the board hangs with only the red LED. 4 of 5 warm
# reboots hung that way; cold boots never did. With the loader in SPI the ROM never touches
# the SD card, and U-Boot resets the card properly itself.
#
# Undo (stock SPI, USB-first): sudo flashcp -v /home/viking/spi-backup-20261114.img /dev/mtd0
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
cd "$(dirname "$0")"

STOCK=/usr/lib/u-boot/rkspi_loader.img
STOCK_ITB=/usr/lib/u-boot/u-boot.itb
NEW_ITB=u-boot-sdfirst.itb
OUT=/tmp/rkspi_loader-sdfirst.img
OFF=$((0x800000))

fail() { echo "ABORT: $*  (SPI not touched)"; exit 1; }
[ -f "$NEW_ITB" ] || fail "$NEW_ITB missing"
grep -qaF 'boot_targets=mmc1 usb0 nvme0' "$NEW_ITB" || fail "$NEW_ITB is not the SD-first image"
[ "$(stat -c %s "$NEW_ITB")" = "$(stat -c %s "$STOCK_ITB")" ] || fail "size mismatch"
[ "$(stat -c %s "$STOCK")" = 16777216 ] || fail "$STOCK is not 16 MiB"
grep -q '"loader"' /proc/mtd && [ "$(awk '$1=="mtd0:"{print $2}' /proc/mtd)" = 01000000 ] || fail "mtd0 is not the 16 MiB 'loader' SPI flash"
cmp -s -n "$(stat -c %s "$STOCK_ITB")" -i $OFF:0 "$STOCK" "$STOCK_ITB" || fail "stock SPI image doesn't hold u-boot.itb at 0x800000"

cp "$STOCK" "$OUT"
dd if="$NEW_ITB" of="$OUT" bs=1 seek=$OFF conv=notrunc status=none
[ "$(cmp -l "$STOCK" "$OUT" | wc -l)" = 40 ] || fail "patched image differs from stock in an unexpected number of bytes"
echo "built $OUT (stock + SD-first U-Boot, 40 bytes differ)"

# flashcp erases, writes and verifies (read-back compare)
flashcp -v "$OUT" /dev/mtd0
cmp "$OUT" /dev/mtd0 && echo "SPI read-back matches"
sync
echo "DONE. Bootloader is in SPI again (SD-first order). Test: sudo reboot  (a warm reboot)"
