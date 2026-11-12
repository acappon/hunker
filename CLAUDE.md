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
  - `/dev/gpiochip*` and `/dev/i2c-*` are root-only (`crw------- root`), and `viking` is not in any gpio/i2c group. Fix with udev rules plus groups, or run as root.
  - The IMU config `sw/ros_ws/src/bno08x_ros2_driver/config/bno085_i2c.yaml` uses `/dev/i2c-1`, which **does not exist** on this Orange Pi. The buses present are 0, 2, 6, 7, 9 and 10. It is left over from the Raspberry Pi 5.
  - **I2C plan (2026-11-11, not done yet):** none of the existing buses reaches the 26-pin header. Use I2C5-M3: SDA = GPIO1_B7, SCL = GPIO1_B6 (from this Pi's device tree). These should be header pins 3 and 5 (from memory of the OPi5 pinout; verify against the silkscreen). Power from pin 1 (3.3 V), GND from pin 6. Enable it with `U_BOOT_FDT_OVERLAYS="device-tree/rockchip/overlay/rk3588-i2c5-m3.dtbo"` in `/etc/default/u-boot`, then `sudo u-boot-update` and reboot. It should then appear as `/dev/i2c-5` (alias i2c5 → fead0000). After that, set `/dev/i2c-5` in the yaml. The IMU wires aren't connected yet; the user will trace them in the IMU case first.
  - `MyGpio` opens gpiochip 4 with Raspberry Pi 5 pin numbers. It only drives the enable and fault LEDs; `enableWheelMotors()` is empty.
  - No game controller was connected: no `/dev/input/js*`, and Bluetooth was inactive.
- Safety guard: if no `/imu` message has arrived within 100 ms, `BalanceDrive` forces the balance term to 0, so only the joystick drives the wheels. Without this guard, a missing IMU gives tilt = 0 and therefore a constant −0.069 duty as soon as the robot is enabled.

## Rules for Claude
- Ask before using `sudo`.
- **Never send motor commands** (duty cycle, heartbeat/enable, or anything else that could spin a wheel) unless the user has confirmed that the wheel is clear and the battery is on. Listening with `candump` is fine.
