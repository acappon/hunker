# Starts everything the robot needs: game controller (joy), BNO085 IMU, and RobotNode.
# Run via /home/viking/hunker/run_hunker.sh (sources both workspaces), by hand or from hunker.service.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    imu_launch = os.path.join(
        get_package_share_directory('bno08x_ros2_driver'), 'launch', 'bno085_i2c.launch.py')

    return LaunchDescription([
        # Publishes /joy. RobotNode treats the controller as disconnected (and e-stops if
        # enabled) when no /joy message arrives for 2 s; autorepeat keeps messages flowing
        # while the sticks are still.
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05}],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_launch)),
        Node(
            package='robot',
            executable='RobotNode',
            name='RobotNode',
            output='screen',
        ),
    ])
