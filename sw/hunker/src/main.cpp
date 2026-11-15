#include "common.h"

std::shared_ptr<RobotNode> g_myRobotNode = nullptr;

int main(int argc, char *argv[])
{
    // rclcpp::init() installs rclcpp's own SIGINT/SIGTERM handler, which only asks spin()
    // to return. All cleanup happens below, in normal code, after spin() has returned and
    // no callback can be running.
    //
    // (Before 2026-11-15 a custom handler destroyed the node from inside the signal
    // handler, while spin() could still be using it, and main() then called
    // emergencyStop() through the null pointer: SIGSEGV on every Ctrl-C.)
    rclcpp::init(argc, argv);
    g_myRobotNode = std::make_shared<RobotNode>();
    g_myRobotNode->init();

    rclcpp::spin(g_myRobotNode);

    g_myRobotNode->writeLog("Shutting down...");
    g_myRobotNode->emergencyStop();

    // Destroying the node destroys BalanceDrive, which sends the "all disabled" CAN
    // heartbeat, and MyGpio, which turns the enable lamp off and releases the lights.
    g_myRobotNode.reset();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
