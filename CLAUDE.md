# Hunker — two-wheel balancing robot

## Hardware
- **Computer:** Orange Pi 5 (RK3588S), Ubuntu 24.04. Pi runs from a 5V/4A wall wart.
- **Motor controllers:** two REV SPARK MAX on CAN. They use the FRC CAN protocol at 1 Mbit/s with 29-bit extended IDs.
  - CAN IDs and firmware are managed with **REV Hardware Client 2** on an Ubuntu laptop, connected to the SPARK MAX by USB-C. The original Windows-only client has reached end of life.
  - CAN IDs: left = 1, right = 2. This matches `sw/hunker/src/BalanceDrive.cpp`.
  - Firmware, 2026-11-11:
    - **Both controllers run 26.1.5.** They were briefly upgraded to 27.0.0, then downgraded.
    - Both controllers must run the same version.
    - REV's spec for 26.x is REV-Specs `spark-frames-2.1.0`; see the driver section below.
  - Avoid 27.0.0 for now. It is **alpha** firmware for the 2027 season: https://github.com/wpilibsuite/SystemCoreTesting/blob/main/REV.md
    - It has no published SPARK CAN spec.
    - It changes units and the duty-cycle parameters.
  - A SPARK MAX keeps its output disabled unless it receives a **continuous enable heartbeat**.
  - Check heartbeat and frame formats against **REV's published spec for the installed firmware version**. Don't rely on older community docs.
- **CAN adapters:**
  - DSD TECH SH-C30L (CANable-compatible, USB). This is the one wired to the SPARK MAX.
    - Verified 2026-11-11: it runs **candleLight firmware**. It shows up as USB `1d50:606f` with the `gs_usb` driver and appears as **`can0`**. No slcan, no `/dev/ttyACM*`.
  - Waveshare SN65HVD230 board on the GPIO header. This is only a transceiver, with no CAN controller of its own. It needs an RK3588 CAN device-tree overlay before it appears as an interface. **As of 2026-11-11 it does not appear as any interface.** `can0` is the USB adapter.
- **Bus:** needs 120 Ω termination at both ends. The SPARK MAX has no built-in termination.
- **Power/ground:** motors run from a 12V battery that is only on during motor tests. A ground wire from the CAN adapter GND to the battery negative is being added.
- **IMU:** BNO085, read on the Pi.
- **WiFi (field link), 2026-11-11:** TP-Link AX900 (Archer TX10UB Nano), USB `3625:010b`, RTL8851BU.
  - Driver: Realtek vendor driver from https://github.com/miftahfaridhh/rtl8851bu (commit `de6d926`; its README lists `3625:010b`). Installed with DKMS as `rtl8851bu/1.0` from `/usr/src/rtl8851bu-1.0`, using our own `dkms.conf` (`AUTOINSTALL=yes`). Module `8851bu`, which autoloads by USB alias.
  - The kernel creates `wlan0`, which udev renames to **`wlx20e15d9e979c`** (MAC-based). The NM profile is bound to that name, so don't rename it without updating the profile.
  - NetworkManager profile **`Hotspot`**: AP mode, WPA-PSK, `ipv4.method shared`. The Pi is **10.42.0.1**, and the laptop gets 10.42.0.x by DHCP. It was created in the desktop GUI and verified with a laptop. It has `autoconnect yes` and priority 10, so it **comes up at boot** with no GUI needed.
  - The user's home WiFi profile `SpectrumSetup-5CA1` is set to `autoconnect no` **on purpose. The user wants the Pi to come up only as a hotspot.** Don't re-enable it.
  - Changing an NM connection over SSH needs `sudo nmcli …`; without it you get "Insufficient privileges".
  - The driver logs at debug level 4 by default, about 4,700 kernel log lines a minute while scanning (quiet in AP mode). `/etc/modprobe.d/8851bu.conf` sets `rtw_drv_log_level=0 rtw_phl_log_level=0`, which takes effect from the next module load.
  - **Slow boot with the dongle plugged in: FIXED 2026-11-14.** It now boots in normal time with the dongle and the CANable plugged in, and the root filesystem mounts at 18 s.
    - **Cause:** at power-up the dongle shows up as a fake USB CD-ROM (`0bda:1a2b`, "RTK Driver Storage", Realtek ZeroCD). The kernel's built-in `usb-storage` and `sr` drivers attached it as `sr0` and kept resetting it every 30 s, 13 times. The root filesystem only mounted at 382 s.
      - Without a console this looked like a hang. Power-cycling before 6 minutes meant no journal was ever written.
      - **Lesson: check the kernel timestamps (`journalctl -b <id> -k -o short-monotonic`) of a slow boot before blaming the bootloader.**
    - **Fix:** `usb-storage.quirks=0bda:1a2b:i` (ignore the device) in `/etc/kernel/cmdline`, applied by `boot/install-boot-config.sh` and `u-boot-update`. The log now shows "usb-storage 3-1:1.0: device ignored". `usb_modeswitch` still switches the dongle to `3625:010b`, because it uses libusb and doesn't need usb-storage.
    - **Also changed, though it turned out not to be the cause: the bootloader now boots from the SD card first.**
      - Stock U-Boot 2017.09 (`u-boot-orangepi-5`, jjriek PPA) has `boot_targets=usb0 mmc1 …` compiled in. It was in SPI flash, which the boot ROM tries before the SD card.
      - `boot/patch-uboot-bootorder.py` swaps the order to `mmc1 usb0` and fixes the FIT sha256; the FIT is unsigned.
      - `boot/install-sdfirst-uboot.sh` wrote the patched image to SD sector 16384 and **erased the SPI flash**, so the board now boots U-Boot from the SD card. That saves about 7 s per boot (Linux starts at 13.8 s instead of about 21 s).
      - Backups on the Pi:
        - `~/spi-backup-20261114.img`, identical to `/usr/lib/u-boot/rkspi_loader.img`
        - `~/sd-bootarea-backup-20261114-204020.img`
      - Undo commands are in the script's header.
      - **If a `u-boot-orangepi-5` package update rewrites the SD bootloader, re-run the patcher and the install script.**
  - **Bluetooth in the AX900 does NOT work on this 6.1 kernel (2026-11-14):**
    - The adapter `hci0` shows up (Realtek, BT 5.3, "Powered: yes"), but a scan finds nothing at all.
    - Cause 1: `btusb`'s Realtek table has no entry for TP-Link's ID `3625:010b`, so it binds as a generic device, with no Realtek setup and no firmware.
    - Cause 2: a test copy of `btusb` with `0bda:b85b` → `3625:010b` (a temporary insmod, nothing installed) got further: "RTL: lmp_subver=8851, loading rtl_bt/rtl8851bu_fw.bin". Then it failed with **"bad EPATCH signature"**. `rtl8851bu_fw.bin` is in Realtek's **v2 format (`RTBTCore`)**, and this kernel's `btrtl` only parses v1 (`Realtech`).
    - The fix would mean backporting a newer btrtl and btusb as DKMS, or a newer kernel.
    - **Plan: a separate Bluetooth dongle.** The TP-Link UB500 (RTL8761BU, USB `2357:0604`) is fully supported here: the ID is in btusb, btrtl knows rtl8761bu, and its firmware is v1. Then ignore or disable the AX900's `hci`.
  - Harmless log noise: missing `/lib/firmware/TXPWR_*.txt`, `[BTC]` coexistence errors (Bluetooth is on a separate interface, handled by `btusb`), and `6ghz domain fail`. Do **not** blacklist `btusb`, even though that driver's README says to; a Bluetooth gamepad may need it.

## Control design
- A 200 Hz real-time thread does the following:
  - reads the IMU
  - runs the balance (pitch) PID and the steering (yaw) PID
  - sends left/right duty-cycle commands and the enable heartbeat over CAN
  - checks a tilt cutoff
- A slower main loop sets targets.
- The SPARK MAX's own PID is optional. Velocity mode may come later.
- No WPILib and no roboRIO/SystemCore.
- **Decision, 2026-11-11: everything stays in C++**, including the 200 Hz loop and the CAN driver. The robot code is C++/ROS2 in `sw/hunker`. The earlier idea of a Python driver is dropped.
  - **2026-11-11: `BalanceDrive` now uses our own driver** (see "SPARK MAX CAN driver" below). `sparkcan` is no longer linked.
    - `BalanceDrive::init()` opens `can0`. It writes no parameters and never burns flash.
    - `BalanceDrive::periodic()` is called once at the end of every `Robot::robotFunction()` (7 ms timer).
      - It polls CAN, then sends the heartbeat and the L/R duty cycles.
      - The motors are enabled only when the robot is enabled **and** both SPARKs have reported Status 0 within 100 ms.
      - States that don't call `update()` get duty 0.
    - The duty sent is hard-limited to `kDutyLimit` = 0.10 (balance + joystick combined) until the motor type and encoder are confirmed.
    - Not built yet:
      - the tilt cutoff
      - a check that IMU data is fresh
      - the 200 Hz dedicated thread from the control design (the loop still runs from the ROS timer)
  - For one-off bench tests, use `cansend` and `candump`.

## History
- Earlier attempts produced no motor control at all. The cause is **still unknown**.
- The obvious suspect would be the vendored `sparkcan` library, since it was written for firmware 24.0.x. **But it isn't the cause.** Its heartbeat (class 11 index 2, `FF`×8) and duty-cycle frame (class 0 index 2) match REV's 26.x spec.
- What `sparkcan` *does* get wrong:
  - It decodes status from the legacy class 6 frames.
  - It uses parameter type float = 2, where the spec says 3.
  - Its BurnFlash uses class 63 index 2; the spec's Persist Parameters is class 63 index 15.
- Other candidates:
  - **Most likely: wrong bitrate.** The user believes `can0` was often up during earlier testing but **without the bitrate set to 1 Mbit/s**. At a wrong bitrate, no frame is acknowledged and the SPARK can't decode anything. At the start of the 2026-11-11 session `can0` was DOWN with no bitrate set, and nothing configures it at boot.
  - `BalanceDrive::init()` writes parameters through `sparkcan` and burns flash on every start, which may throw or block.
  - `kP`/`kF` in `BalanceDrive.cpp` are declared `int`, so they truncate to 0.
  - An earlier version used REVLib (`rev::spark`), which needs a WPILib HAL to run.

## SPARK MAX CAN driver (ours)
- Files:
  - `sw/hunker/src/SparkMaxCan.{h,cpp}`: classes `spark::CanBus` and `spark::SparkMax`
  - `sw/hunker/tools/spark_bench.cpp`: bench tool, built as the `spark_bench` target
- **Source of truth: REV's official CAN spec.** https://github.com/REVrobotics/REV-Specs, file `can-frames/spark-frames-2.1.0`, commit 1e90305 (2026-01-02). It covers firmware 25.x and 26.x.
- What the driver implements:
  - **Secondary Heartbeat:** ID `0x02052C80`, 8-byte little-endian u64. Bit n enables CAN ID n; 0 disables all. It only works while Status 0 bit 53 "Primary Heartbeat Lock" is 0, which it is on our units. The SPARK times out about 100 ms after the last heartbeat.
  - **Duty Cycle Setpoint:** class 0 index 2, 8 bytes, float LE.
  - **Set Statuses Enabled:** class 1 index 0, 4 bytes.
  - Decoding of Status 0, 1, 2 and the Unique ID Broadcast.
- The driver does **not** write parameters or burn flash. Configure the controllers with REV Hardware Client 2 instead.
- **2026-11-11: first motor control on this robot worked.** Command: `spark_bench spin 2 0.05 3 --wheel-clear`, with the belt off and no load.
  - The right motor turned.
  - Applied output followed the command, ending at about 0.041, and returned to 0 on stop.
  - Current was 0.0–0.1 A, with no faults and no bus errors.
  - **Heartbeat bit n = CAN ID n is confirmed.**
  - After the reboot, ±0.05 duty runs were repeated, 2 s each. The user saw the motor spin **both directions**. The applied output was about ±0.042. Current stayed at about 0 A, with no faults.
- **Left (ID 1) at ±0.05 duty, 2 s each, behaves the same as the right.**
  - Applied output was about ±0.041, current 0.0–0.2 A, with no faults, and it returned to 0 on stop.
  - Its unique ID is `0x66A26D4F`.
  - The user saw the left motor spin both directions too.
- **Open issue, on BOTH controllers: Status 2 is all `FF` bytes, so velocity and position read NaN.** The controller is reporting no primary-encoder data. Possible causes:
  - the motor type is set to brushed
  - the encoder cable is disconnected
  - the motor is not a NEO
  - Check in REV Hardware Client 2 before raising duty. A NEO running in brushed mode can draw very high current: the IGVC project saw about 30 A at 8 % duty.
- Status 0 arrives every 20 ms on our units, while the spec's default is 10 ms.
- Community reference for 26.1.5 without a roboRIO: https://github.com/Paarseus/IGVC_ROS2/pull/24 (Teensy firmware).

## Bus check, 2026-11-11 (wiring and bitrate are confirmed OK)
- The SPARK MAX stayed **silent until it saw traffic on the bus**. After one 11-bit test frame (`cansend can0 123#00`) it began transmitting. The test frame was acknowledged, and there were no errors.
- Only **device ID 2** (the right wheel) was seen. The left SPARK MAX (ID 1) had no power during this test. Frames observed:
  - `0x0205B802` every 20 ms (API 0x2E0)
  - `0x0205B842` every 250 ms (API 0x2E1)
  - `0x0205BC02` once (API 0x2F0)
- Re-checked on 26.1.5 with **both controllers powered**. Both answer, with identical frame layouts:
  - `0x0205B80n` every 20 ms
  - `0x0205B84n` every 250 ms
  - `0x0205BC0n` every 1 s, with a constant per-device payload (probably a serial number or unique ID): ID 1 sends `4F6DA266`, ID 2 sends `9C79B175`
  - There were no bus errors.
- Decoded against the spec:
  - API 0x2E0 is Status 0 (class 46 index 0).
  - API 0x2E1 is Status 1.
  - API 0x2F0 is the Unique ID Broadcast (class 47).
  - `sparkcan` reads the legacy status frames (class 6) instead, so its status values are wrong. Its heartbeat and duty frames are correct.

## CAN bring-up (needs sudo — ask first)
```
sudo apt install can-utils
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
candump -t a -x can0        # SPARK MAX status frames should appear when 12V is on
```
**Persistent config since 2026-11-11:** `/etc/systemd/network/80-can0.network` sets `BitRate=1M` and `RestartSec=100ms`. `can0` now comes up at 1 Mbit/s after a reboot, which was verified.

To check it: `ip -details link show can0 | grep -E "state|bitrate"`. You should see `bitrate 1000000` and `ERROR-ACTIVE`.

## Running the robot software
- `/home/viking/hunker/run_hunker.sh` sources ROS, `sw/ros_ws` (IMU driver) and `sw/hunker`. It then launches `sw/hunker/launch/hunker_all.launch.py`, which starts:
  - `joy_node`, which publishes `/joy` (autorepeat 20 Hz)
  - the BNO085 driver
  - `RobotNode`
- **Initial testing (user's choice, 2026-11-11):** Claude starts and stops the program for each test. The motors stay disabled until the controller's B button enables the robot.
- **Later: start at power-up.** `/home/viking/hunker/hunker.service` is a systemd unit that runs `run_hunker.sh` as user `viking`.
  - It is written but **not installed**. Install commands are in the file's header.
  - `sta.sh`, `stop.sh` and `live_log.sh` already expect the name `hunker.service`.
- Blockers found 2026-11-11, still unresolved:
  - **IMU working, 2026-11-14:**
    - The BNO085 is at 0x4A on `/dev/i2c-5` (I2C5-M3).
    - Wiring on the OPi5 header: pin 1 = 3.3 V, pin 3 = SDA (GPIO1_B7), pin 5 = SCL (GPIO1_B6), pin 9 = GND, **pin 7 = INT** (GPIO1_C6 = `gpiochip1` line 22). INT on pin 7 is confirmed: interrupts arrive.
    - `/etc/default/u-boot` has `U_BOOT_FDT_OVERLAYS="device-tree/rockchip/overlay/rk3588-i2c5-m3.dtbo device-tree/rockchip/overlay/hunker-i2c5-400k.dtbo"`. The second overlay is ours: it runs I2C5 at 400 kHz. Source is in `boot/`, installed by `sudo boot/install-boot-config.sh`; **re-run that script after a kernel upgrade**.
      - The path must be **relative to `/lib/firmware/<kernel>/`**. `u-boot-update` silently skips absolute paths and files it can't find.
      - Check with `grep fdtoverlays /boot/extlinux/extlinux.conf`.
    - Permissions: `/dev/i2c-*` belongs to group `i2c` (rule from the i2c-tools package). `/dev/gpiochip*` belongs to group `gpio` through `/etc/udev/rules.d/60-gpio.rules`. `viking` is in both groups.
    - The yaml now has `bus: /dev/i2c-5`, `gpio_chip: gpiochip1`, `gpio_line: 22`. The driver needs INT: it only reads after an INT falling edge.
    - **`/imu` runs at 300 Hz** (jitter about 1 ms, worst gap 8 ms). Fixed 2026-11-14:
      - The driver never called `gpiod_line_event_read()`, so it busy-polled I2C. It now consumes the event and re-polls while INT stays low.
      - It now publishes on every rotation-vector report, instead of waiting for one of each report. The accelerometer only manages about 70 Hz and had capped the rate.
      - It now fills `header.stamp` correctly.
      - The 400 kHz bus and the rtprio limit (`sysconfig/`) are in place. There were zero I2C read errors.
    - **The accelerometer reads 13.3 m/s² at rest (about 1.36× too high, direction correct).** Probably a bad BNO085 accel calibration. The fused orientation isn't affected.
    - **`BalanceDrive` now uses the fused orientation, not `accel.x`:**
      - tilt = asin(u.x), where u = world "up" in the IMU frame. That's the same axis and sign as the old accel.x signal.
      - Tilt rate is computed exactly from the gyro.
      - `kBalanceAngle` = 0.779 rad comes from the old accel.x = 6.89. Given the accel scale error, it may really be about 31°. **Re-measure it with a hand-tilt test.**
      - kP = 0.07/rad (equivalent to the old 0.01 per m/s²); kD = 0.
      - At rest the IMU's x axis points straight up, and fused tilt agrees with accel tilt (89.5° vs 90°).
      - **The hand-tilt test hasn't been done yet.** It's needed to confirm the sign of tilt and tilt rate and to set the balance angle.
  - **Status lights (2026-11-14):** `MyGpio` now opens **gpiochip1**.
    - Enable lamp: **header pin 16** = GPIO1_D3 = line 27. It drives a transistor base, and the lamp itself runs from 12 V. A 10 kΩ base-to-emitter pull-down is recommended so it stays off at power-up.
    - Fault LED: **pin 18** = GPIO1_D2 = line 26. It's an assembly with a built-in resistor.
    - GND: pin 14.
    - Test the wiring with `sw/hunker/tools/led_test` (`gcc … -lgpiod`). It blinks each light 3 times.
    - **The board would not boot with the lights connected, on pins 11/13 and again on 16/18:** red LED only, no green heartbeat, no journal.
      - **Probable cause: the enable circuit's input wire sits at about 4.5 V** when disconnected (user measured it). The fault wire reads millivolts, which is fine.
        - The reading is the same with the 12 V battery on or off, so the source isn't 12 V.
        - 4.5 V ≈ 5 V minus one junction drop, which suggests a transistor emitter or diode on a live 5 V supply, probably the Pi's 5 V pin (2/4) or USB 5 V.
        - The circuit, per the user: an NPN with its **emitter on battery ground**, collector → enable lamp → +12 V, and the base (through a resistor, presumably) on the enable GPIO. The fault LED runs from the fault GPIO to **battery ground**. Both are fine for 3.3 V logic.
        - **Likely real cause: battery ground isn't tied to Pi ground** (that wire was "being added"). Then battery ground floats several volts away from the Pi, and the GPIO sees that offset. Next: measure battery negative to a Pi GND pin. If it isn't ~0 V, add the battery-negative ↔ Pi-GND wire; CAN needs it too.
        - Open question: which reference the 4.5 V was measured against.
      - 5 V on a 3.3 V GPIO back-feeds the RK3588 I/O supply through the pin's protection diode and upsets power-up sequencing. The Raspberry Pi 5 isn't 5 V tolerant either; it probably tolerated it out of spec.
      - **Don't connect the enable wire until it reads about 0 V open-circuit.** Remove the pull-up / 5 V source and use a plain NPN with a 10 kΩ base-emitter pull-down, or a logic-level MOSFET.
      - An earlier guess (11/13 double as DDR PHY debug pins `ddrphych0/1/2`) is probably wrong. Still, prefer 16/18; pin 22 is shared with eMMC.
    - `enableWheelMotors()` is still empty.
  - No game controller was connected: no `/dev/input/js*`, and Bluetooth was inactive.
- Safety guard: if no `/imu` message has arrived within 100 ms, `BalanceDrive` forces the balance term to 0, so only the joystick drives the wheels. Without this guard, a missing IMU gives tilt = 0 and therefore a constant −0.069 duty as soon as the robot is enabled.

## Backups
- **SD card image, 2026-11-14** (CAN + IMU working, before the WiFi/BT boot-order work), on the user's laptop:
  `/mnt/EAGLEHEAD/acappon/Docs/Hunker_images/OrangePi5/20261114_CAN_IMU_working_before_DebugWifi_andBT.img.gz`
  - Made with `sudo dd if=/dev/sdX bs=4M status=progress | gzip -1 > <file>`, with the card in the laptop (it was `/dev/sde` that time).
  - Restore: `gunzip -c <file> | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync`. **Check `/dev/sdX` with `lsblk` first**, because the name can change between plug-ins.
- Code and config: GitHub `acappon/hunker`. System files placed by hand are copied in `sysconfig/`.

## Rules for Claude
- Ask before using `sudo`.
- **Never send motor commands** (duty cycle, heartbeat/enable, or anything else that could spin a wheel) unless the user has confirmed that the wheel is clear and the battery is on. Listening with `candump` is fine.
