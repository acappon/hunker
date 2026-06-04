
#include <csignal>
#include <cstdio>

#include "common.h"

std::shared_ptr<RobotNode> g_myRobotNode = nullptr;

void signal_handler(int signum)
{
    try
    {
        if (g_myRobotNode)
        {
            g_myRobotNode->writeLog("Signal %d received, shutting down...", signum);
            g_myRobotNode->emergencyStop();
            g_myRobotNode.reset();
        }
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "Exception during signal handling: %s\n", e.what());
    }
    catch (...)
    {
        std::fprintf(stderr, "Unknown exception during signal handling\n");
    }
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);

    try
    {
        g_myRobotNode = std::make_shared<RobotNode>();
        g_myRobotNode->init();
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "Fatal: failed to initialize RobotNode: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(g_myRobotNode);

    g_myRobotNode->emergencyStop();
    g_myRobotNode.reset();
    sleep(1);

    rclcpp::shutdown();
    return 0;
}