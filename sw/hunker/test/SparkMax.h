#pragma once
// Stub for REVLib SparkMax motor controller

namespace rev
{
enum class ResetMode { kResetSafeParameters };
enum class PersistMode { kNoPersistParameters };

namespace spark
{

class SparkMaxConfig
{
public:
    enum class IdleMode { kCoast, kBrake };
    SparkMaxConfig &SetIdleMode(IdleMode) { return *this; }
};

class SparkMax
{
public:
    enum class MotorType { kBrushless, kBrushed };

    SparkMax(int canId, MotorType) : m_canId(canId), m_power(0.0) {}

    void Configure(const SparkMaxConfig &, ResetMode, PersistMode) {}
    void Set(double power) { m_power = power; }
    double Get() const { return m_power; }

private:
    int m_canId;
    double m_power;
};

} // namespace spark
} // namespace rev
