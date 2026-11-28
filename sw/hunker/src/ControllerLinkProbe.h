#ifndef CONTROLLER_LINK_PROBE_H
#define CONTROLLER_LINK_PROBE_H

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

struct DBusConnection;

// Fast check that the Bluetooth controller is really still there.
//
// Why: the Xbox controller only sends HID reports when something changes, and joy_node
// autorepeats the last state at 20 Hz. A dead controller therefore looks connected until
// BlueZ's link supervision timeout (~3-4 s) removes the device. (Measured 2026-11-27:
// 0 raw reports in 5 s while idle, 109/s while moving a stick.)
//
// How: a background thread reads the controller's GATT Battery Level characteristic
// (UUID 0x2A19) through BlueZ over D-Bus, one read at a time, every kPeriodMs. A read
// is a request/reply over the radio, so the replies stop as soon as the controller does.
// Round trip measured 25-33 ms typical, 162 ms worst (with busctl).
// The robot code checks secondsSinceLastReply() and disables the motors if it's too old.
//
// Fail-safe: until the first reply arrives (or if the characteristic can't be found),
// secondsSinceLastReply() is huge, so the robot can't be enabled.
class ControllerLinkProbe
{
public:
    ControllerLinkProbe() = default;
    ~ControllerLinkProbe();

    void start();
    void stop();

    // Seconds since the last successful read reply (very large if none yet).
    double secondsSinceLastReply() const;

    // Human-readable state, for logging when it changes.
    std::string status() const;

    static constexpr int kPeriodMs = 100;        // one read every 100 ms
    static constexpr int kCallTimeoutMs = 1000;  // give up on a single read after 1 s

private:
    void threadMain();
    bool findCharacteristic();   // sets m_charPath; true if found
    bool readOnce(std::string &error);
    void setStatus(const std::string &s);

    std::thread m_thread;
    std::atomic<bool> m_stop{false};
    DBusConnection *m_conn = nullptr;            // private connection, used only by the thread
    std::string m_charPath;                      // e.g. /org/bluez/hci0/dev_.../service0012/char0013

    std::atomic<int64_t> m_lastReplyNs{0};       // steady_clock ns; 0 = never

    mutable std::mutex m_statusMutex;
    std::string m_status = "starting";
};

#endif // CONTROLLER_LINK_PROBE_H
