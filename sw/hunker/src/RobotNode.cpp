#include "common.h"


extern std::shared_ptr<RobotNode> g_myRobotNode;

RobotNode::RobotNode()
    : Node("RobotNode"), m_isControllerConnected(false), m_isRobotEnabled(false)
{
}

RobotNode::~RobotNode()
{
    m_linkProbe.stop();
    m_joystick_sub.reset();
    m_feedback_publisher.reset();
    m_imu_sub.reset();
    m_safetyTimer.reset();
    m_robotTimer.reset();
}

void RobotNode::init()
{
    std::string sRet;

    try
    {
        m_isControllerConnected = false;
        m_isRobotEnabled = false;
        m_isRobotEmergencyStopped = false;

        m_myGpio.initEnableAndFaultLED();
        m_robot.init();
        m_linkProbe.start();

        m_last_joy_msg_time = m_last_imu_msg_time = this->now() - rclcpp::Duration(10, 0);

        // Subscribe to joystick messages
        auto qos_joy = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_joy.keep_last(10)
            .best_effort()
            .durability_volatile();
        m_joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", qos_joy,
            std::bind(&RobotNode::joy_callback, this, std::placeholders::_1));

        // Create a publisher for the feedback topic
        m_feedback_publisher = this->create_publisher<sensor_msgs::msg::JoyFeedback>("joy/set_feedback", 10);


        // Create a subscription to the /imu topic
        auto qos_imu = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_imu.keep_last(10)
            .best_effort()
            .durability_volatile();
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos_imu,
            std::bind(&RobotNode::imu_callback, this, std::placeholders::_1));

        m_safetyTimer = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&RobotNode::safetyFunction, this));

        // 7ms interval corresponds to 143hz.  The IMU sends data at approx 150hz, so this update rate in the robot wiil 
        // get at least 1 update per cycle of the IMU data, and will be fast enough to provide good control responsiveness.
        // Note that this design has an outer PID loop at 143hz, and an inner PID loop in the SparkMAX motor controllers a 1Khz.
        // The outer loop sets velocity for the motors, and the inner loop converges on that velocity.
        
        m_robotTimer = this->create_wall_timer(
            std::chrono::milliseconds(7),   
            std::bind(&RobotNode::robotFunction, this));
    }
    catch (const std::exception &e)
    {
        writeLog("Exception in RobotNode::init()");
        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
        writeLog("%s", e.what());
    }
}

void RobotNode::writeLog(const char *fmt, ...)
{
    try
    {
        char buf[1001];
        va_list vl;
        va_start(vl, fmt);
        int len = std::vsnprintf(buf, sizeof buf, fmt, vl);
        va_end(vl);
        if (len <= 0)
        {
            return; // Empty message or formatting error
        }

        if (m_lastLogMsg == buf)
        {
            ++m_logRepeatCount; // Exact repeat: count it instead of printing it
            return;
        }

        // First column: how many repeats of the previous message were suppressed
        RCLCPP_INFO(this->get_logger(), "%5u | %s%s", m_logRepeatCount, buf,
                    (len >= static_cast<int>(sizeof buf)) ? " [truncated]" : "");
        m_lastLogMsg = buf;
        m_logRepeatCount = 0;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception in RobotNode::writeLog: %s", e.what());
        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
    }
}

bool RobotNode::isImuFresh(double maxAgeSec)
{
    return (this->now() - m_last_imu_msg_time).seconds() <= maxAgeSec;
}

bool RobotNode::isRobotEnabled()
{
    return m_isRobotEnabled;
}

void RobotNode::safetyFunction()
{
    try
    {
        if (m_isRobotEmergencyStopped)
        {
            emergencyStop();
        }
        else
        {
            checkControllerConnection();
        }

        if (isRobotEnabled() && (!m_isControllerConnected))
        {
            m_isRobotEmergencyStopped = true;
        }

        updateLEDs();

        checkRobotEnableDisable();
    }
    catch (const std::exception &e)
    {
        writeLog("Exception in RobotNode::safetyFunction");
        writeLog("%s", e.what());
        writeLog("Stack trace: %s", getStackTrace().c_str());

        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
    }
}

void RobotNode::robotFunction()
{
    try
    {
        // Controller link check every 7 ms cycle, before the robot code runs, so a stale
        // link means duty 0 in this same cycle. Disable only (no latch): press B once the
        // link is back. A real disconnect is still handled by safetyFunction().
        if (m_isRobotEnabled && !isControllerLinkOk())
        {
            writeLog("Controller link stale: no probe reply for %.0f ms -> Robot DISABLED (press B to re-enable)",
                     m_linkProbe.secondsSinceLastReply() * 1000.0);
            enableRobot(false);
        }

        m_robot.robotFunction();
    }
    catch (const std::exception &e)
    {
        writeLog("Exception in Robot::robotFunction");
        writeLog("%s", e.what());
        writeLog("Stack trace: %s", getStackTrace().c_str());

        m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_EXCEPTION, true);
    }
}

void RobotNode::emergencyStop()
{
    m_isRobotEmergencyStopped = true;
    m_isRobotEnabled = false;
    m_myGpio.enableWheelMotors(false);
}

void RobotNode::enableRobot(bool isEnabled)
{
    if (m_isRobotEmergencyStopped)
    {
        // Cycle power to clear emergency stop
        return;
    }
    m_isRobotEnabled = isEnabled;
    m_myGpio.enableWheelMotors(isEnabled);
}

void RobotNode::checkControllerConnection()
{
    rclcpp::Time now = this->now();
    bool prev = m_isControllerConnected;
    // joy_node publishes at least 20 Hz (autorepeat_rate), so 0.5 s of silence = 10 missed
    // messages in a row. (Was 2 s; a moving robot shouldn't run uncontrolled that long.)
    if (std::fabs((now - m_last_joy_msg_time).seconds()) > kControllerTimeoutSec)
    {
        writeLog("Controller is disconnected, now = %ld, last = %ld",
                 now.nanoseconds(),
                 m_last_joy_msg_time.nanoseconds());
        m_isControllerConnected = false;
        // Drop any press latched before the loss.
        std::fill(std::begin(m_joy_pressed), std::end(m_joy_pressed), false);
    }
    else
    {
        m_isControllerConnected = true;
    }

    const bool linkOk = isControllerLinkOk();
    if (linkOk != m_lastLinkOk)
    {
        writeLog("Controller link probe: %s", linkOk ? "replies OK" : "STALE (no reply)");
        m_lastLinkOk = linkOk;
    }
    const std::string probeStatus = m_linkProbe.status();
    if (probeStatus != m_lastProbeStatus && probeStatus.rfind("OK", 0) != 0)
    {
        writeLog("Controller link probe status: %s", probeStatus.c_str());   // problems only; "OK" is logged above
    }
    m_lastProbeStatus = probeStatus;

    // Fault LED "controller" (1 blink) for a lost controller or a stale link.
    m_faultIndicator.setFault(FaultIndicator::FAULT_TYPE::FAULT_CONTROLLER_DISCONNECTED,
                              !m_isControllerConnected || !linkOk);

    if (prev != m_isControllerConnected)
    {
        if (m_isControllerConnected)
        {
            writeLog("Controller is connected");
        }
        else
        {
            writeLog("Controller is disconnected");
        }
    }
}

void RobotNode::checkRobotEnableDisable()
{
    // Take all latched presses each cycle, so an old press can't act later.
    const bool pressedB = takeButtonPress(JOY_B);
    const bool pressedX = takeButtonPress(JOY_X);
    const bool pressedLB = takeButtonPress(JOY_LB);
    const bool pressedRB = takeButtonPress(JOY_RB);

    if (pressedB)
    {
        if (!isRobotEnabled())
        {
            // Sticks must be centred, so nothing moves the moment the robot enables.
            // (The sticks read 0 from the first /joy message with this joy_node + xpadneo, and
            // the arrays start at 0, so no "move the sticks first" step is needed.)
            if (!isControllerLinkOk())
            {
                writeLog("Not enabling: controller link not confirmed (%s)", m_linkProbe.status().c_str());
            }
            else if (!areAllJoysticksInDeadzone())
            {
                writeLog("Not enabling: sticks not centred");
                sendXboxRumble();
            }
            else
            {
                writeLog("Robot ENABLED");
                enableRobot(true);
            }
        }
    }

    if (pressedX)
    {
        if (isRobotEnabled())
        {
            writeLog("Robot DISABLED");
            enableRobot(false);
        }
    }

    // Emergency stop on a press, or while LB/RB is held.
    if (pressedLB || pressedRB || m_joy_buttons[JOY_LB] || m_joy_buttons[JOY_RB])
    {
        if (!m_isRobotEmergencyStopped)
        {
            writeLog("EMERGENCY STOP (LB/RB)");
        }
        emergencyStop();
    }
}

void RobotNode::updateLEDs()
{
    if (m_isRobotEmergencyStopped)
    {
        writeLog("updateLEDs: ESTOP");

        m_myGpio.setEnableLED(false);
        m_myGpio.setFaultLED(true);
    }
    else
    {
        if (!m_isControllerConnected)
        {
            writeLog("updateLEDs: XBox NOT connected");
            m_myGpio.setEnableLED(false);
        }
        else
        {
            if (!m_isRobotEnabled)
            {
                writeLog("updateLEDs: Robot disabled");
                m_myGpio.setEnableLED(true);
            }
            else
            {
                std::string sMsg = "updateLEDs: Robot ENABLED, state ";
                sMsg += m_robot.getRobotStateText();
                writeLog(sMsg);
                flashEnableLED();
            }
        }
        m_faultIndicator.update();
    }
}

void RobotNode::flashEnableLED()
{
    int millisecs = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count() %
                    1000;
    m_myGpio.setEnableLED(millisecs < 500);
}

std::string RobotNode::getStackTrace()
{
    try
    {
        {
            void *array[100];
            int size;

            // Get the backtrace
            size = backtrace(array, 10);
            return *backtrace_symbols(array, size);
        }
    }
    catch (const std::exception &e)
    {
        // ignore exception
    }
    return "EXCEPTION in getStatckTrace()";
}

void RobotNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Listen for Joystick messages
    /*
    writeLog("Axes: [%f, %f, %f, %f, %f, %f]",
             msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3],
             msg->axes[4], msg->axes[5]);
    writeLog("Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
             msg->buttons[0], msg->buttons[1], msg->buttons[2],
             msg->buttons[3], msg->buttons[4], msg->buttons[5],
             msg->buttons[6], msg->buttons[7], msg->buttons[8],
             msg->buttons[9], msg->buttons[10], msg->buttons[11]);
    */

    m_isControllerConnected = true;

    m_last_joy_msg_time = this->now();

    // writeLog("controller message received, %s", std::to_string(m_last_joy_msg_time.seconds()).c_str());

    // Store the joystick data.
    // Layout measured 2026-11-15: Xbox Series controller over Bluetooth, xpadneo v0.10 driver,
    // ROS 2 Jazzy joy_node (reports it as "Xbox 360 Controller", 9 axes, 11 buttons).
    //   axes:    0 LX, 1 LY, 2 LT, 3 RX, 4 RY, 5 RT, 6 (unused, rests at 1.0), 7/8 D-pad
    //            triggers rest at +1.0 and go to -1.0 when fully pressed
    //   buttons: 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 View/Back, 7 Menu/Start, 8 Xbox, 9 L3, 10 R3
    // (The old layout, from the Raspberry Pi setup, had X=3, Y=4, LB=6, RB=7 and the right
    // stick on axes 2/3, so X, LB/RB and the deadzone check didn't work with this controller.)
    if (msg->axes.size() < 6 || msg->buttons.size() < 11)
    {
        writeLog("Unexpected /joy layout: %zu axes, %zu buttons; ignoring message",
                 msg->axes.size(), msg->buttons.size());
        return;
    }

    bool previous[NUMBER_OF_JOY_BUTTONS];
    std::copy(std::begin(m_joy_buttons), std::end(m_joy_buttons), std::begin(previous));

    m_joy_axes[LJOY_LEFT_RIGHT] = msg->axes[0];
    m_joy_axes[LJOY_FWD_BACK] = msg->axes[1];
    m_joy_axes[LTRIGGER] = msg->axes[2];
    m_joy_axes[RJOY_LEFT_RIGHT] = msg->axes[3];
    m_joy_axes[RJOY_FWD_BACK] = msg->axes[4];
    m_joy_axes[RTRIGGER] = msg->axes[5];

    m_joy_buttons[JOY_A] = msg->buttons[0];
    m_joy_buttons[JOY_B] = msg->buttons[1];
    m_joy_buttons[JOY_X] = msg->buttons[2];
    m_joy_buttons[JOY_Y] = msg->buttons[3];
    m_joy_buttons[JOY_LB] = msg->buttons[4];
    m_joy_buttons[JOY_RB] = msg->buttons[5];
    m_joy_buttons[JOY_BACK] = msg->buttons[6];
    m_joy_buttons[JOY_START] = msg->buttons[7];
    m_joy_buttons[JOY_LOGITECH] = msg->buttons[8];   // the Xbox button
    m_joy_buttons[JOY_L3] = msg->buttons[9];
    m_joy_buttons[JOY_R3] = msg->buttons[10];

    // Latch presses (button went down) for the slower safety timer.
    for (int i = 0; i < NUMBER_OF_JOY_BUTTONS; ++i)
    {
        if (m_joy_buttons[i] && !previous[i])
        {
            m_joy_pressed[i] = true;
        }
    }

    // Emergency stop acts immediately, not on the next 250 ms safety tick.
    // (checkRobotEnableDisable() also checks LB/RB, as a backup.)
    if ((m_joy_buttons[JOY_LB] || m_joy_buttons[JOY_RB]) && !m_isRobotEmergencyStopped)
    {
        writeLog("EMERGENCY STOP (LB/RB)");
        emergencyStop();
    }
}

bool RobotNode::takeButtonPress(JOY_BUTTONS button)
{
    const bool pressed = m_joy_pressed[button];
    m_joy_pressed[button] = false;
    return pressed && m_isControllerConnected;   // never act on a press from a lost controller
}

// Callback function for IMU messages
void RobotNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_last_imu_msg_time = this->now();

    m_imu_orientation[IMU_AXES::W] = msg->orientation.w;
    m_imu_orientation[IMU_AXES::X] = msg->orientation.x;
    m_imu_orientation[IMU_AXES::Y] = msg->orientation.y;
    m_imu_orientation[IMU_AXES::Z] = msg->orientation.z;

    m_imu_linear_acceleration[IMU_AXES::X] = msg->linear_acceleration.x;
    m_imu_linear_acceleration[IMU_AXES::Y] = msg->linear_acceleration.y;
    m_imu_linear_acceleration[IMU_AXES::Z] = msg->linear_acceleration.z;

    m_imu_angular_velocity[IMU_AXES::X] = msg->angular_velocity.x;
    m_imu_angular_velocity[IMU_AXES::Y] = msg->angular_velocity.y;
    m_imu_angular_velocity[IMU_AXES::Z] = msg->angular_velocity.z;
}

void RobotNode::sendXboxRumble()
{
    double vibe_intensity = 0.5; // Vibration intensity (0.0 to 1.0)

    // Left rumble
    sensor_msgs::msg::JoyFeedback feedback_msg_left;
    feedback_msg_left.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    feedback_msg_left.id = 0; // 0 for left rumble
    feedback_msg_left.intensity = vibe_intensity;
    m_feedback_publisher->publish(feedback_msg_left);

    // Right rumble
    sensor_msgs::msg::JoyFeedback feedback_msg_right;
    feedback_msg_right.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    feedback_msg_right.id = 1; // 1 for right rumble
    feedback_msg_right.intensity = vibe_intensity;
    m_feedback_publisher->publish(feedback_msg_right);
}

bool RobotNode::areAllJoysticksInDeadzone()
{
    // Check if all joystick axes,(but not the trigger axes), are within the deadzone
    for (int i = 0; i < 4; ++i)
    {
        if (std::fabs(g_myRobotNode->m_joy_axes[i]) > JOYSTICK_DEADZONE)
        {
            return false;
        }
    }
    return true;
}
