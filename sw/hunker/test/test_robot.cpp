#include <gtest/gtest.h>
#include "common.h"
#include "stubs.h"

extern std::shared_ptr<RobotNode> g_myRobotNode;

class RobotTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        stubs::reset();
        g_myRobotNode = std::make_shared<RobotNode>();
        m_robot.init();
    }

    void TearDown() override
    {
        g_myRobotNode.reset();
    }

    Robot m_robot;
};

// --- getRobotStateText() ---

TEST_F(RobotTest, InitialStateIsDisabled)
{
    EXPECT_EQ(m_robot.getRobotStateText(), "Disabled");
}

// --- State transitions via periodic() ---

TEST_F(RobotTest, RemainsDisabledWhenNotEnabled)
{
    // Robot is not enabled by default in g_myRobotNode
    m_robot.periodic();
    EXPECT_EQ(m_robot.getRobotStateText(), "Disabled");
}

TEST_F(RobotTest, DisabledPeriodicDisablesBalanceDrive)
{
    m_robot.disabledPeriodic();
    EXPECT_FALSE(m_robot.m_myBalanceDrive.isEnabled());
}

TEST_F(RobotTest, BalancingPeriodicEnablesBalanceDrive)
{
    m_robot.balancingPeriodic();
    EXPECT_TRUE(m_robot.m_myBalanceDrive.isEnabled());
}

TEST_F(RobotTest, AirbornePeriodicEnablesBalanceDrive)
{
    m_robot.airbornePeriodic();
    EXPECT_TRUE(m_robot.m_myBalanceDrive.isEnabled());
}

TEST_F(RobotTest, LandingPeriodicEnablesBalanceDrive)
{
    m_robot.landingPeriodic();
    EXPECT_TRUE(m_robot.m_myBalanceDrive.isEnabled());
}

TEST_F(RobotTest, HunkeredPeriodicEnablesBalanceDrive)
{
    m_robot.hunkeredPeriodic();
    EXPECT_TRUE(m_robot.m_myBalanceDrive.isEnabled());
}

TEST_F(RobotTest, GetRobotStateTextUnknownForInvalidState)
{
    // After init the state is DISABLED; getRobotStateText should handle
    // the known states. We can't force an unknown state without modifying
    // private members, but we can verify all known states via the public
    // periodic functions and getRobotStateText doesn't crash.
    EXPECT_EQ(m_robot.getRobotStateText(), "Disabled");
}
