#include <gtest/gtest.h>
#include "common.h"
#include "stubs.h"

extern std::shared_ptr<RobotNode> g_myRobotNode;

class FaultIndicatorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        stubs::reset();
        g_myRobotNode = std::make_shared<RobotNode>();
    }

    void TearDown() override
    {
        g_myRobotNode.reset();
    }
};

TEST_F(FaultIndicatorTest, InitiallyNoFaultsActive)
{
    FaultIndicator fi;
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_NONE));
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_CONTROLLER_DISCONNECTED));
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_EXCEPTION));
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));
}

TEST_F(FaultIndicatorTest, SetFaultMakesItActive)
{
    FaultIndicator fi;
    fi.setFault(FaultIndicator::FAULT_WATCHDOG, true);
    EXPECT_TRUE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_EXCEPTION));
}

TEST_F(FaultIndicatorTest, ClearFaultMakesItInactive)
{
    FaultIndicator fi;
    fi.setFault(FaultIndicator::FAULT_WATCHDOG, true);
    EXPECT_TRUE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));

    fi.setFault(FaultIndicator::FAULT_WATCHDOG, false);
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));
}

TEST_F(FaultIndicatorTest, MultipleFaultsCanBeActiveSimultaneously)
{
    FaultIndicator fi;
    fi.setFault(FaultIndicator::FAULT_WATCHDOG, true);
    fi.setFault(FaultIndicator::FAULT_EXCEPTION, true);
    fi.setFault(FaultIndicator::FAULT_CONTROLLER_DISCONNECTED, true);

    EXPECT_TRUE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));
    EXPECT_TRUE(fi.isActive(FaultIndicator::FAULT_EXCEPTION));
    EXPECT_TRUE(fi.isActive(FaultIndicator::FAULT_CONTROLLER_DISCONNECTED));
}

TEST_F(FaultIndicatorTest, ClearOneFaultLeavesOthersActive)
{
    FaultIndicator fi;
    fi.setFault(FaultIndicator::FAULT_WATCHDOG, true);
    fi.setFault(FaultIndicator::FAULT_EXCEPTION, true);

    fi.setFault(FaultIndicator::FAULT_WATCHDOG, false);
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));
    EXPECT_TRUE(fi.isActive(FaultIndicator::FAULT_EXCEPTION));
}

TEST_F(FaultIndicatorTest, SetFaultIdempotent)
{
    FaultIndicator fi;
    fi.setFault(FaultIndicator::FAULT_WATCHDOG, true);
    fi.setFault(FaultIndicator::FAULT_WATCHDOG, true);
    EXPECT_TRUE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));

    fi.setFault(FaultIndicator::FAULT_WATCHDOG, false);
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));
}

TEST_F(FaultIndicatorTest, ClearInactiveFaultIsNoOp)
{
    FaultIndicator fi;
    fi.setFault(FaultIndicator::FAULT_WATCHDOG, false);
    EXPECT_FALSE(fi.isActive(FaultIndicator::FAULT_WATCHDOG));
}

TEST_F(FaultIndicatorTest, UpdateWithNoFaultsTurnsOffLED)
{
    FaultIndicator fi;
    fi.update();
    // Should have written GPIO_LED_FAULT (pin 4) to 0 (off)
    EXPECT_EQ(stubs::lastGpio(), MyGpio::GPIO_LED_FAULT);
    EXPECT_EQ(stubs::lastValue(), 0);
}
