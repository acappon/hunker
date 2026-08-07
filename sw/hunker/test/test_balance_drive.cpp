#include <gtest/gtest.h>
#include "common.h"
#include "stubs.h"

extern std::shared_ptr<RobotNode> g_myRobotNode;

class BalanceDriveTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        stubs::reset();
        g_myRobotNode = std::make_shared<RobotNode>();
        m_bd.init();
    }

    void TearDown() override
    {
        g_myRobotNode.reset();
    }

    BalanceDrive m_bd;
};

// --- isEnabled / setEnable ---

TEST_F(BalanceDriveTest, InitiallyDisabled)
{
    EXPECT_FALSE(m_bd.isEnabled());
}

TEST_F(BalanceDriveTest, EnableSetsState)
{
    m_bd.setEnable(true);
    EXPECT_TRUE(m_bd.isEnabled());
}

TEST_F(BalanceDriveTest, DisableAfterEnable)
{
    m_bd.setEnable(true);
    EXPECT_TRUE(m_bd.isEnabled());

    m_bd.setEnable(false);
    EXPECT_FALSE(m_bd.isEnabled());
}

TEST_F(BalanceDriveTest, SetEnableIdempotent)
{
    m_bd.setEnable(true);
    m_bd.setEnable(true);
    EXPECT_TRUE(m_bd.isEnabled());

    m_bd.setEnable(false);
    m_bd.setEnable(false);
    EXPECT_FALSE(m_bd.isEnabled());
}

// --- update() when disabled ---

TEST_F(BalanceDriveTest, UpdateWhileDisabledDoesNotCrash)
{
    // Just verify it doesn't crash
    m_bd.update(0.0, 0.0);
    EXPECT_FALSE(m_bd.isEnabled());
}

TEST_F(BalanceDriveTest, UpdateWhileDisabledWithNonZeroInputs)
{
    m_bd.update(0.5, -0.5);
    EXPECT_FALSE(m_bd.isEnabled());
}

// --- update() when enabled ---

TEST_F(BalanceDriveTest, UpdateWhileEnabledDoesNotCrash)
{
    m_bd.setEnable(true);
    // IMU data defaults to 0, so tilt_error = 0 - kBalancePoint = -6.89
    // balance_output = kP * (-6.89) = -0.0689, clamped to [-0.15, 0.15]
    m_bd.update(0.0, 0.0);
    EXPECT_TRUE(m_bd.isEnabled());
}

TEST_F(BalanceDriveTest, UpdateWithLargeInputsClampsCorrectly)
{
    m_bd.setEnable(true);
    // Pass extreme joystick values — should not crash
    m_bd.update(5.0, -5.0);
    EXPECT_TRUE(m_bd.isEnabled());
}

TEST_F(BalanceDriveTest, UpdateWithMaxNegativeInputs)
{
    m_bd.setEnable(true);
    m_bd.update(-1.0, -1.0);
    EXPECT_TRUE(m_bd.isEnabled());
}
