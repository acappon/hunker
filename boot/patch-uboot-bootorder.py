#!/usr/bin/env python3
"""Make the Orange Pi 5 vendor U-Boot (2017.09, package u-boot-orangepi-5) try the SD
card before USB.

Why: its built-in boot_targets start with usb0. A USB WiFi dongle that powers up as a
fake CD-ROM (Realtek ZeroCD, 0bda:1a2b) makes U-Boot's USB storage scan stall for
~5 minutes before it falls back to the SD card (mmc1).

How: swap "usb0 mmc1" -> "mmc1 usb0" in the default environment string (same length,
so nothing in the image moves), then rewrite the U-Boot image's sha256 in the FIT
header. The FIT is unsigned (signature node has no value), so only the per-image hashes
are checked.

Usage: patch-uboot-bootorder.py IN.itb OUT.itb
"""
import hashlib
import subprocess
import sys
import tempfile

OLD = b"boot_targets=usb0 mmc1 nvme0"
NEW = b"boot_targets=mmc1 usb0 nvme0"


def fdtget(dtb, *args):
    return subprocess.run(["fdtget", dtb, *args], check=True, capture_output=True, text=True).stdout.split()


def main(src_path, dst_path):
    src = open(src_path, "rb").read()
    hdr_size = int.from_bytes(src[4:8], "big")          # FDT totalsize
    data_base = (hdr_size + 0x1FF) & ~0x1FF               # external data, 512-byte aligned

    with tempfile.NamedTemporaryFile(suffix=".dtb") as t:
        t.write(src[:hdr_size]); t.flush()
        off = int(fdtget(t.name, "-t", "u", "/images/uboot", "data-offset")[0])
        size = int(fdtget(t.name, "-t", "u", "/images/uboot", "data-size")[0])
        stored = bytes.fromhex("".join(w.zfill(8) for w in fdtget(t.name, "-t", "x", "/images/uboot/hash", "value")))
        if fdtget(t.name, "-p", "/configurations/conf/signature").count("value"):
            sys.exit("FIT is signed; refusing to patch")

    start, end = data_base + off, data_base + off + size
    if hashlib.sha256(src[start:end]).digest() != stored:
        sys.exit("input U-Boot hash does not match its FIT header; wrong layout?")
    if src.count(OLD) != 1:
        sys.exit(f"expected exactly one {OLD!r}, found {src.count(OLD)}")
    i = src.index(OLD)
    if not start <= i < end:
        sys.exit("boot_targets string is not inside the U-Boot image")

    out = bytearray(src)
    out[i:i + len(OLD)] = NEW
    new_hash = hashlib.sha256(bytes(out[start:end])).digest()
    if bytes(out[:hdr_size]).count(stored) != 1:
        sys.exit("could not locate the U-Boot hash uniquely in the FIT header")
    j = bytes(out[:hdr_size]).index(stored)
    out[j:j + 32] = new_hash
    open(dst_path, "wb").write(out)
    print(f"patched {dst_path}: boot order at byte {i}, hash at byte {j}, new sha256 {new_hash.hex()}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
