
class FaultIndicator
{
public:
    FaultIndicator();
    ~FaultIndicator();

    typedef enum
    {
        FAULT_NONE,
        FAULT_CONTROLLER_DISCONNECTED,
        FAULT_EXCEPTION,
        FAULT_WATCHDOG,
        NUMBER_OF_FAULTS
    } FAULT_TYPE;
    void setFault(FaultIndicator::FAULT_TYPE fault, bool isFault);
    bool isActive(FaultIndicator::FAULT_TYPE fault);

    void update();

private:
    void resetDisplaySequence();

    // When either of these are -1, the fault indicator is paused
    int m_faultIdx = 0;   // Index of the fault being displayed
    int m_faultCount = 0; // Number of times the fault has been flashed for the current fault index

    std::vector<FAULT_TYPE> m_faults;
    std::chrono::_V2::steady_clock::time_point m_lastCountTime;       // Since the start of displaying a fault
    std::chrono::_V2::steady_clock::time_point m_faultCountStartTime; // Since the start of the current count
};