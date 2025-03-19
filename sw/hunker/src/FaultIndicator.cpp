
#include <chrono>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "MyGpio.hpp"
#include "FaultIndicator.hpp"
#include "Motor.h"
#include "IMU_bno055.h"
#include "Robot.h"
#include "RobotNode.hpp"

extern std::shared_ptr<RobotNode> g_myRobotNode;

FaultIndicator::FaultIndicator()
{
    m_faults = std::vector<FAULT_TYPE>();

    m_faultIdx = 0;   // Index of the fault being displayed
    m_faultCount = 0; // Number of times the fault has been flashed for the current fault index

    m_faults = std::vector<FAULT_TYPE>();
    m_lastCountTime = std::chrono::steady_clock::now();
    m_faultCountStartTime = std::chrono::steady_clock::now();
}

FaultIndicator::~FaultIndicator()
{
    m_faults.clear();
}

void FaultIndicator::setFault(FAULT_TYPE fault, bool isFault)
{
    if (isActive(fault) == isFault)
    {
        // No change
        return;
    }
    if (isFault)
    {
        m_faults.push_back(fault);
    }
    else
    {
        auto it = std::find(m_faults.begin(), m_faults.end(), fault);
        if (it != m_faults.end())
        {
            m_faults.erase(it);
        }
    }

    // Reset display to start whenever any error status is added or subtracted
    m_faultCountStartTime = std::chrono::steady_clock::now();
    m_lastCountTime = std::chrono::steady_clock::now();
    m_faultIdx = 0;
    m_faultCount = 0;
}

bool FaultIndicator::isActive(FAULT_TYPE fault)
{
    return std::find(m_faults.begin(), m_faults.end(), fault) != m_faults.end();
}

void FaultIndicator::resetDisplaySequence()
{
    // Reset display to start whenever any error status is added or subtracted
    m_faultCountStartTime = std::chrono::steady_clock::now();
    m_lastCountTime = std::chrono::steady_clock::now();
    m_faultIdx = 0;
    m_faultCount = 0;
}

void FaultIndicator::update()
{
    auto timeDiff = std::chrono::steady_clock::now() - m_lastCountTime;
    int ms_since_last_count =
        std::chrono::duration_cast<std::chrono::milliseconds>(timeDiff).count();

    if (m_faults.size() == 0)
    {
        g_myRobotNode->m_myGpio.setFaultLED(false);
        return;
    }

    if (m_faultCount == -1)
    {
        if (ms_since_last_count > 2000)
        {
            m_faultCount = 0;
            m_faultIdx++;
            if (m_faultIdx >= m_faults.size())
            {
                m_faultIdx = -1;
            }
            m_lastCountTime = std::chrono::steady_clock::now();
        }
    }
    else if (m_faultIdx == -1)
    {
        if (ms_since_last_count > 5000)
        {
            resetDisplaySequence();
        }
    }
    else if (ms_since_last_count < 500)
    {
        g_myRobotNode->m_myGpio.setFaultLED(true);
    }
    else if (ms_since_last_count > 1000)
    {
        m_faultCount++;
        if (m_faultCount >= m_faults[m_faultIdx])
        {
            m_faultCount = -1;
        }
        m_lastCountTime = std::chrono::steady_clock::now();
    }
    else if (ms_since_last_count > 500)
    {
        g_myRobotNode->m_myGpio.setFaultLED(false);
    }
}
