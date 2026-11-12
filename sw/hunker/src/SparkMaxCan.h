// Minimal SocketCAN driver for REV SPARK MAX, firmware 26.1.x.
//
// Every frame here is taken from REV's official CAN spec:
//   https://github.com/REVrobotics/REV-Specs  can-frames/spark-frames-2.1.0  (commit 1e90305)
// and checked against candump captures of our own controllers (see CLAUDE.md).
//
// Deliberately small: heartbeat, duty-cycle setpoint, status-frame enable, and
// decoding of Status 0/1/2. No parameter writes, no flash burns. Configure the
// controllers with REV Hardware Client 2 instead.
//
// Threading: NOT thread-safe. Call everything (poll, heartbeat, setpoints) from
// the one control thread. Nothing here blocks: a full TX queue drops the frame
// and counts it.
//
// Safety model:
//  - A SPARK only drives its output while it receives an enabling heartbeat.
//    It times out ~100 ms after the last one, then disables itself.
//    So if this process stalls or dies, the motors stop on their own.
//  - Send the heartbeat every control cycle (at least every 20 ms).
//  - To stop: sendHeartbeat(0) and duty 0. CanBus's destructor does this too.

#ifndef SPARKMAXCAN_H
#define SPARKMAXCAN_H

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

struct can_frame;

namespace spark
{

// FRC CAN 29-bit arbitration ID: type(5) | manufacturer(8) | apiClass(6) | apiIndex(4) | device(6)
constexpr uint32_t kDeviceTypeMotorController = 2;
constexpr uint32_t kManufacturerREV = 5;

constexpr uint32_t makeArbId(uint32_t apiClass, uint32_t apiIndex, uint32_t deviceId)
{
    return (kDeviceTypeMotorController << 24) | (kManufacturerREV << 16) |
           ((apiClass & 0x3F) << 10) | ((apiIndex & 0x0F) << 6) | (deviceId & 0x3F);
}

// Frame API class/index, from spark-frames-2.1.0
constexpr uint32_t kApiDutyCycleSetpoint[2] = {0, 2};    // 8 bytes, float duty -1..1
constexpr uint32_t kApiSetStatusesEnabled[2] = {1, 0};   // 4 bytes, u16 mask, u16 enabled
constexpr uint32_t kApiSecondaryHeartbeat[2] = {11, 2};  // 8 bytes, u64 enabled-SPARKs bitfield, device id 0
constexpr uint32_t kApiStatusClass = 46;                 // Status 0..9 = index 0..9
constexpr uint32_t kApiUniqueIdClass = 47;               // Unique ID Broadcast = index 0

struct Status0
{
    float appliedOutput = 0;   // -1..1
    float busVoltage = 0;      // V
    float current = 0;         // A
    uint8_t motorTempC = 0;
    bool inverted = false;
    bool primaryHeartbeatLock = false;   // true: SPARK ignores the Secondary Heartbeat until power-cycled
    uint8_t sparkModel = 0;              // 2 = SPARK MAX on our units
};

struct Status1
{
    uint8_t activeFaults = 0;     // bit0 other, 1 motor type, 2 sensor, 3 CAN, 4 temp, 5 DRV, 6 ESC EEPROM, 7 firmware
    uint8_t activeWarnings = 0;   // bit0 brownout, 1 overcurrent, 2 ESC EEPROM, 3 ext EEPROM, 4 sensor, 5 stall, 6 has reset, 7 other
    uint8_t stickyFaults = 0;     // same bit layout as activeFaults
    uint8_t stickyWarnings = 0;   // same bit layout as activeWarnings
    bool isFollower = false;
};

struct Status2
{
    float velocityRpm = 0;        // primary encoder (NEO hall sensor), RPM
    float positionRotations = 0;  // primary encoder, rotations
};

class SparkMax;

class CanBus
{
public:
    explicit CanBus(const std::string& interfaceName);   // throws std::runtime_error
    ~CanBus();   // if an enabling heartbeat was ever sent: sends disable heartbeats, then closes
    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    // Read all pending frames and update the registered SparkMax objects. Never blocks.
    void poll();

    // Secondary Heartbeat. Bit n enables the SPARK with CAN ID n; 0 disables all.
    bool sendHeartbeat(uint64_t enabledDevices);
    static constexpr uint64_t deviceBit(int canId) { return uint64_t{1} << (canId & 0x3F); }

    // Send an 11-bit standard frame that no SPARK acts on. Only to wake a silent bus for monitoring.
    bool sendWakeFrame();

    bool send(uint32_t arbId, const uint8_t* data, uint8_t len);   // 29-bit extended frame

    uint32_t txDropped() const { return m_txDropped; }

private:
    friend class SparkMax;
    void registerDevice(SparkMax* dev);
    void unregisterDevice(SparkMax* dev);
    void dispatch(const can_frame& frame);

    int m_socket = -1;
    std::array<SparkMax*, 64> m_devices{};
    uint32_t m_txDropped = 0;
    bool m_everEnabled = false;   // an enabling heartbeat was sent; destructors then send disable frames
};

class SparkMax
{
public:
    using Clock = std::chrono::steady_clock;

    SparkMax(CanBus& bus, int canId);
    ~SparkMax();
    SparkMax(const SparkMax&) = delete;
    SparkMax& operator=(const SparkMax&) = delete;

    int canId() const { return m_canId; }

    // Duty cycle, clamped to [-1, 1]; NaN/inf is sent as 0.
    // Has no effect unless this SPARK's bit is set in the heartbeat.
    bool setDutyCycle(float duty);

    // Status 2 (encoder velocity/position) is off by default. Enables Status 0, 1, 2.
    bool enableEncoderStatus();

    // Latest decoded status, and how old it is. ageMs() = -1 if never received.
    const Status0& status0() const { return m_status0; }
    const Status1& status1() const { return m_status1; }
    const Status2& status2() const { return m_status2; }
    uint32_t uniqueId() const { return m_uniqueId; }
    long status0AgeMs() const { return ageMs(m_status0Time); }
    long status2AgeMs() const { return ageMs(m_status2Time); }
    bool isAlive(long maxAgeMs = 100) const;   // Status 0 seen within maxAgeMs

private:
    friend class CanBus;
    void handleFrame(uint32_t apiClass, uint32_t apiIndex, const uint8_t* data, uint8_t len);
    static long ageMs(Clock::time_point t);

    CanBus& m_bus;
    int m_canId;
    Status0 m_status0;
    Status1 m_status1;
    Status2 m_status2;
    uint32_t m_uniqueId = 0;
    Clock::time_point m_status0Time{};
    Clock::time_point m_status2Time{};
};

}   // namespace spark

#endif   // SPARKMAXCAN_H
