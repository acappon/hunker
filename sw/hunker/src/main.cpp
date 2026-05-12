
#include <csignal>

#include "common.h"

std::shared_ptr<RobotNode> g_myRobotNode = nullptr;

void signal_handler(int signum)
{
    try
    {
        g_myRobotNode->writeLog("Signal %d received, shutting down...", signum);
        g_myRobotNode->emergencyStop();
        g_myRobotNode.reset(); // explicitly destroy the node
    }
    catch (const std::exception &e)
    {
        // ignore exception
    }
    rclcpp::shutdown(); // signals spin() to return, then main() cleans up
}

int main(int argc, char *argv[])
{
    // Register signal(s) handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);
    g_myRobotNode = std::make_shared<RobotNode>();
    g_myRobotNode->init();

    rclcpp::spin(g_myRobotNode);

    g_myRobotNode->emergencyStop();
    g_myRobotNode.reset(); // explicitly destroy the node
    sleep(1);

    rclcpp::shutdown();
    return 0;
}