// Bench tool for SparkMaxCan. Standalone: no ROS.
//
//   spark_bench monitor <id>[,<id>...] [seconds]
//       Listen only and print decoded status. Sends only an 11-bit wake frame,
//       which no SPARK acts on. Never enables a motor.
//
//   spark_bench spin <id> <duty> <seconds> --wheel-clear
//       Enable ONE SPARK and run it at a duty cycle for a few seconds, then stop.
//       |duty| <= 0.20 and seconds <= 10 are enforced. Ramps up over 0.5 s.
//       Aborts if the SPARK is silent, faulted, or locked to a primary heartbeat,
//       or if its status stops arriving. Ctrl-C stops it at once.
//
// How it stops: duty 0 plus a disabling heartbeat (bitfield 0) for 200 ms.
// If this process is killed hard instead, the SPARK disables itself ~100 ms
// after the last heartbeat.

#include "../src/SparkMaxCan.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

static volatile std::sig_atomic_t g_stop = 0;
static void onSignal(int) { g_stop = 1; }

static constexpr float kMaxBenchDuty = 0.20f;
static constexpr double kMaxBenchSeconds = 10.0;
static constexpr auto kCycle = 10ms;   // heartbeat + setpoint period

static void printStatus(const spark::SparkMax& m)
{
    const auto& s0 = m.status0();
    const auto& s1 = m.status1();
    long age = m.status0AgeMs();
    if (age < 0)
    {
        std::printf("  id %d: no status yet\n", m.canId());
        return;
    }
    std::printf("  id %d: out=%+.3f  %.2fV  %.1fA  %uC  model=%u lock=%d inv=%d | "
                "faults=%02X warn=%02X sticky f=%02X w=%02X | ",
                m.canId(), s0.appliedOutput, s0.busVoltage, s0.current, s0.motorTempC, s0.sparkModel,
                s0.primaryHeartbeatLock, s0.inverted, s1.activeFaults, s1.activeWarnings, s1.stickyFaults,
                s1.stickyWarnings);
    if (m.status2AgeMs() >= 0 && m.status2AgeMs() < 200)
        std::printf("%+.0f rpm %+.2f rot | ", m.status2().velocityRpm, m.status2().positionRotations);
    std::printf("uid=%08X age=%ldms\n", m.uniqueId(), age);
}

static int usage()
{
    std::fprintf(stderr,
                 "usage: spark_bench [-i can0] monitor <id>[,<id>...] [seconds]\n"
                 "       spark_bench [-i can0] spin <id> <duty> <seconds> --wheel-clear\n");
    return 2;
}

static int monitor(spark::CanBus& bus, const std::string& ids, double seconds)
{
    std::vector<std::unique_ptr<spark::SparkMax>> motors;
    std::stringstream ss(ids);
    for (std::string tok; std::getline(ss, tok, ',');)
        motors.push_back(std::make_unique<spark::SparkMax>(bus, std::atoi(tok.c_str())));

    bus.sendWakeFrame();
    auto end = Clock::now() + std::chrono::duration<double>(seconds);
    auto nextPrint = Clock::now() + 200ms;
    while (!g_stop && Clock::now() < end)
    {
        bus.poll();
        if (Clock::now() >= nextPrint)
        {
            nextPrint += 500ms;
            for (auto& m : motors)
                printStatus(*m);
            std::printf("\n");
        }
        std::this_thread::sleep_for(kCycle);
    }
    return 0;
}

static void stopAll(spark::CanBus& bus, spark::SparkMax& m)
{
    auto end = Clock::now() + 200ms;
    while (Clock::now() < end)
    {
        bus.sendHeartbeat(0);
        m.setDutyCycle(0.0f);
        bus.poll();
        std::this_thread::sleep_for(kCycle);
    }
}

static int spin(spark::CanBus& bus, int id, float duty, double seconds)
{
    if (std::fabs(duty) > kMaxBenchDuty || seconds <= 0 || seconds > kMaxBenchSeconds)
    {
        std::fprintf(stderr, "refused: need |duty| <= %.2f and 0 < seconds <= %.0f\n", kMaxBenchDuty,
                     kMaxBenchSeconds);
        return 2;
    }

    spark::SparkMax m(bus, id);
    m.setDutyCycle(0.0f);

    // 1. Wake the bus and wait for this SPARK's status, with the heartbeat disabled.
    bus.sendWakeFrame();
    auto deadline = Clock::now() + 1500ms;
    while (!m.isAlive() && Clock::now() < deadline && !g_stop)
    {
        bus.sendHeartbeat(0);
        bus.poll();
        std::this_thread::sleep_for(kCycle);
    }
    std::this_thread::sleep_for(300ms);   // let Status 1 (250 ms period) arrive
    bus.poll();
    printStatus(m);
    if (!m.isAlive())
    {
        std::fprintf(stderr, "abort: no Status 0 from CAN ID %d (powered? wired? ID right?)\n", id);
        return 1;
    }
    if (m.status0().primaryHeartbeatLock)
    {
        std::fprintf(stderr, "abort: SPARK %d is locked to a primary heartbeat; power-cycle it\n", id);
        return 1;
    }
    if (m.status1().activeFaults)
    {
        std::fprintf(stderr, "abort: SPARK %d has active faults %02X\n", id, m.status1().activeFaults);
        return 1;
    }
    m.enableEncoderStatus();

    // 2. Enable this one SPARK and ramp to the duty cycle.
    std::printf("spinning CAN ID %d at duty %+.3f for %.1f s (Ctrl-C stops)\n", id, duty, seconds);
    const auto start = Clock::now();
    const auto end = start + std::chrono::duration<double>(seconds);
    auto nextPrint = start + 250ms;
    auto next = start;
    int rc = 0;
    while (!g_stop && Clock::now() < end)
    {
        bus.poll();
        if (m.status0AgeMs() > 200)
        {
            std::fprintf(stderr, "abort: status from CAN ID %d stopped arriving\n", id);
            rc = 1;
            break;
        }
        double t = std::chrono::duration<double>(Clock::now() - start).count();
        float cmd = duty * static_cast<float>(std::fmin(1.0, t / 0.5));
        bus.sendHeartbeat(spark::CanBus::deviceBit(id));
        m.setDutyCycle(cmd);
        if (Clock::now() >= nextPrint)
        {
            nextPrint += 250ms;
            std::printf("cmd=%+.3f ", cmd);
            printStatus(m);
        }
        next += kCycle;
        std::this_thread::sleep_until(next);
    }

    // 3. Stop.
    stopAll(bus, m);
    std::this_thread::sleep_for(300ms);
    bus.poll();
    std::printf("stopped. ");
    printStatus(m);
    if (bus.txDropped())
        std::printf("warning: %u CAN frames could not be sent\n", bus.txDropped());
    return rc;
}

int main(int argc, char** argv)
{
    std::string iface = "can0";
    int a = 1;
    if (argc > 2 && std::strcmp(argv[1], "-i") == 0)
    {
        iface = argv[2];
        a = 3;
    }
    if (argc - a < 2)
        return usage();

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    try
    {
        spark::CanBus bus(iface);
        std::string cmd = argv[a];
        if (cmd == "monitor")
            return monitor(bus, argv[a + 1], argc - a > 2 ? std::atof(argv[a + 2]) : 3.0);
        if (cmd == "spin")
        {
            if (argc - a != 5 || std::strcmp(argv[a + 4], "--wheel-clear") != 0)
                return usage();
            return spin(bus, std::atoi(argv[a + 1]), std::strtof(argv[a + 2], nullptr), std::atof(argv[a + 3]));
        }
        return usage();
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
