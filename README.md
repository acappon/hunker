# Robot Program for Raspberry Pi 5

This project is a ROS2-based robot program designed to run on a Raspberry Pi 5 with Ubuntu 24.04 LTS. It allows for viewing logs and debugging information via ROS topics and supports remote debugging.

## Project Structure

```
robot-program
├── src
│   ├── main.cpp          # Entry point of the robot program
│   ├── RobotNode.cpp    # Implementation of the RobotNode class
│   ├── RobotNode.hpp    # Definition of the RobotNode class
│   └── CMakeLists.txt    # Build configuration for the project
├── launch
│   └── robot_launch.py    # Launch script for the robot node
├── package.xml           # Metadata about the ROS2 package
└── README.md             # Documentation for the project
```

## Setup Instructions

1. **Install ROS2**: Follow the official ROS2 installation guide for Ubuntu 24.04 LTS.
2. **Clone the repository**: 
   ```
   git clone <repository-url>
   cd robot-program
   ```
3. **Build the project**:
   ```
   colcon build
   source install/setup.bash
   ```

## Usage

To launch the robot node, use the following command:
```
ros2 launch robot-program robot_launch.py
```

## Remote Debugging

This project supports remote debugging using SSH. You can set up your Visual Studio Code to connect to the Raspberry Pi 5 via SSH for debugging purposes.

## Additional Information

- Ensure that all dependencies listed in `package.xml` are installed.
- Modify `robot_launch.py` to configure parameters as needed for your specific robot setup.