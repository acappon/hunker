// See SparkMaxCan.h. Bit positions and scale factors are copied from
// REV-Specs can-frames/spark-frames-2.1.0; all signals are little-endian.

#include "SparkMaxCan.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace spark
{

namespace
{

// Little-endian bit field extraction, as the REV spec defines signals.
uint64_t le64(const uint8_t* data, uint8_t len)
{
    uint64_t v = 0;
    for (int i = 0; i < len && i < 8; ++i)
        v |= uint64_t{data[i]} << (8 * i);
    return v;
}

uint64_t bits(uint64_t v, int pos, int len)
{
    return (v >> pos) & ((len >= 64) ? ~uint64_t{0} : ((uint64_t{1} << len) - 1));
}

int64_t signedBits(uint64_t v, int pos, int len)
{
    uint64_t raw = bits(v, pos, len);
    uint64_t sign = uint64_t{1} << (len - 1);
    return static_cast<int64_t>((raw ^ sign) - sign);
}

float floatBits(uint64_t v, int pos)
{
    uint32_t raw = static_cast<uint32_t>(bits(v, pos, 32));
    float f;
    std::memcpy(&f, &raw, sizeof f);
    return f;
}

void putLe32(uint8_t* out, uint32_t v)
{
    for (int i = 0; i < 4; ++i)
        out[i] = static_cast<uint8_t>(v >> (8 * i));
}

}   // namespace

// ---------------------------------------------------------------- CanBus

CanBus::CanBus(const std::string& interfaceName)
{
    m_socket = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (m_socket < 0)
        throw std::runtime_error("CanBus: socket() failed: " + std::string(std::strerror(errno)));

    // Only receive REV motor-controller frames (type 2, manufacturer 5).
    can_filter filter{};
    filter.can_id = (kDeviceTypeMotorController << 24 | kManufacturerREV << 16) | CAN_EFF_FLAG;
    filter.can_mask = 0x1FFF0000u | CAN_EFF_FLAG | CAN_RTR_FLAG;
    ::setsockopt(m_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof filter);

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, interfaceName.c_str(), IFNAMSIZ - 1);
    if (::ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        int e = errno;
        ::close(m_socket);
        throw std::runtime_error("CanBus: no interface " + interfaceName + ": " + std::strerror(e));
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(m_socket, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0)
    {
        int e = errno;
        ::close(m_socket);
        throw std::runtime_error("CanBus: bind " + interfaceName + " failed: " + std::strerror(e));
    }
}

CanBus::~CanBus()
{
    // Best effort: if we ever enabled anything, tell every SPARK to disable and zero
    // the outputs we own. A listen-only user sends nothing here.
    for (int i = 0; m_everEnabled && i < 3; ++i)
    {
        sendHeartbeat(0);
        for (SparkMax* dev : m_devices)
            if (dev)
                dev->setDutyCycle(0.0f);
    }
    ::close(m_socket);
}

bool CanBus::send(uint32_t arbId, const uint8_t* data, uint8_t len)
{
    can_frame frame{};
    frame.can_id = (arbId & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.can_dlc = len > 8 ? 8 : len;
    std::memcpy(frame.data, data, frame.can_dlc);
    if (::write(m_socket, &frame, sizeof frame) != static_cast<ssize_t>(sizeof frame))
    {
        ++m_txDropped;   // EAGAIN/ENOBUFS (TX queue full, bus down) — never block the control loop
        return false;
    }
    return true;
}

bool CanBus::sendHeartbeat(uint64_t enabledDevices)
{
    uint8_t data[8];
    for (int i = 0; i < 8; ++i)
        data[i] = static_cast<uint8_t>(enabledDevices >> (8 * i));
    if (enabledDevices)
        m_everEnabled = true;
    return send(makeArbId(kApiSecondaryHeartbeat[0], kApiSecondaryHeartbeat[1], 0), data, 8);
}

bool CanBus::sendWakeFrame()
{
    can_frame frame{};
    frame.can_id = 0x123;   // standard 11-bit ID: not addressed to any FRC device
    frame.can_dlc = 1;
    return ::write(m_socket, &frame, sizeof frame) == static_cast<ssize_t>(sizeof frame);
}

void CanBus::poll()
{
    can_frame frame;
    while (::read(m_socket, &frame, sizeof frame) == static_cast<ssize_t>(sizeof frame))
        dispatch(frame);
}

void CanBus::dispatch(const can_frame& frame)
{
    if (!(frame.can_id & CAN_EFF_FLAG) || (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)))
        return;
    uint32_t id = frame.can_id & CAN_EFF_MASK;
    if ((id >> 16) != (kDeviceTypeMotorController << 8 | kManufacturerREV))
        return;
    SparkMax* dev = m_devices[id & 0x3F];
    if (dev)
        dev->handleFrame((id >> 10) & 0x3F, (id >> 6) & 0x0F, frame.data, frame.can_dlc);
}

void CanBus::registerDevice(SparkMax* dev)
{
    if (m_devices[dev->canId()])
        throw std::runtime_error("CanBus: CAN ID registered twice: " + std::to_string(dev->canId()));
    m_devices[dev->canId()] = dev;
}

void CanBus::unregisterDevice(SparkMax* dev)
{
    if (m_devices[dev->canId()] == dev)
        m_devices[dev->canId()] = nullptr;
}

// -------------------------------------------------------------- SparkMax

SparkMax::SparkMax(CanBus& bus, int canId) : m_bus(bus), m_canId(canId)
{
    if (canId < 1 || canId > 62)
        throw std::invalid_argument("SparkMax: CAN ID must be 1..62");
    m_bus.registerDevice(this);
}

SparkMax::~SparkMax()
{
    if (m_bus.m_everEnabled)
        setDutyCycle(0.0f);
    m_bus.unregisterDevice(this);
}

bool SparkMax::setDutyCycle(float duty)
{
    if (!std::isfinite(duty))
        duty = 0.0f;
    duty = std::fmax(-1.0f, std::fmin(1.0f, duty));

    // bytes 0-3: float setpoint; 4-5: arbitrary feedforward (0); bits 48-50: PID slot 0,
    // feedforward units 0; rest reserved 0.
    uint8_t data[8] = {};
    uint32_t raw;
    std::memcpy(&raw, &duty, sizeof raw);
    putLe32(data, raw);
    return m_bus.send(makeArbId(kApiDutyCycleSetpoint[0], kApiDutyCycleSetpoint[1], m_canId), data, 8);
}

bool SparkMax::enableEncoderStatus()
{
    // Mask selects which status bits to change; bit n = Status n. Turn on 0, 1, 2.
    constexpr uint16_t mask = 0x0007, enabled = 0x0007;
    uint8_t data[4] = {mask & 0xFF, mask >> 8, enabled & 0xFF, enabled >> 8};
    return m_bus.send(makeArbId(kApiSetStatusesEnabled[0], kApiSetStatusesEnabled[1], m_canId), data, 4);
}

bool SparkMax::isAlive(long maxAgeMs) const
{
    long age = status0AgeMs();
    return age >= 0 && age <= maxAgeMs;
}

long SparkMax::ageMs(Clock::time_point t)
{
    if (t == Clock::time_point{})
        return -1;
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t).count();
}

void SparkMax::handleFrame(uint32_t apiClass, uint32_t apiIndex, const uint8_t* data, uint8_t len)
{
    const uint64_t v = le64(data, len);

    if (apiClass == kApiUniqueIdClass && apiIndex == 0 && len >= 4)
    {
        m_uniqueId = static_cast<uint32_t>(bits(v, 0, 32));
        return;
    }
    if (apiClass != kApiStatusClass || len < 8)
        return;

    switch (apiIndex)
    {
    case 0:
        m_status0.appliedOutput = static_cast<float>(signedBits(v, 0, 16) * 3.082369457075716e-05);
        m_status0.busVoltage = static_cast<float>(bits(v, 16, 12) * 0.0073260073260073);
        m_status0.current = static_cast<float>(bits(v, 28, 12) * 0.0366300366300366);
        m_status0.motorTempC = static_cast<uint8_t>(bits(v, 40, 8));
        m_status0.inverted = bits(v, 52, 1);
        m_status0.primaryHeartbeatLock = bits(v, 53, 1);
        m_status0.sparkModel = static_cast<uint8_t>(bits(v, 54, 4));
        m_status0Time = Clock::now();
        break;
    case 1:
        m_status1.activeFaults = static_cast<uint8_t>(bits(v, 0, 8));
        m_status1.activeWarnings = static_cast<uint8_t>(bits(v, 16, 8));
        m_status1.stickyFaults = static_cast<uint8_t>(bits(v, 24, 8));
        m_status1.stickyWarnings = static_cast<uint8_t>(bits(v, 40, 8));
        m_status1.isFollower = bits(v, 48, 1);
        break;
    case 2:
        m_status2.velocityRpm = floatBits(v, 0);
        m_status2.positionRotations = floatBits(v, 32);
        m_status2Time = Clock::now();
        break;
    default:
        break;
    }
}

}   // namespace spark
