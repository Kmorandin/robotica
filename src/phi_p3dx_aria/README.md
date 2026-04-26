# phi_p3dx_aria

This ROS2 package provides an interface for the Pioneer P3-DX robot using the ARIA library from Adept MobileRobots. It publishes robot data (odometry, sensors) and receives velocity commands.

## Overview

The `phi_p3dx_aria` is based on [ROSARIA](https://github.com/amor-ros-pkg/rosaria) (for ROS1), but has been updated for ROS2 Humble. The main updates include:
- Complete migration to ROS2 (rclcpp, modern messages).
- Support for additional sensors (sonar as LaserScan).
- Compatibility with recent ARIA library versions.
- Integration with tf2 for transformations.

### Features
- **Data publication**: Odometry (`/odom`), laser sensors (`/laser_scan`), sonar (`/sonar`).
- **Velocity control**: Receives commands on `/cmd_vel` and sends to the robot via ARIA.
- **TF transformations**: Publishes TF between base_link and odom.

## Prerequisites

- **ROS2 Humble** (or compatible).
- **ARIA library**: Install ARIA (tested with version 2.7.2)
- Dependencies: `rclcpp`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`.

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/phir2-lab/phi_p3dx_aria
   ```

3. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select phi_p3dx_aria
   source install/setup.bash
   ```

## Usage

### Connection with Real Robot

1. **Configure serial connection** (adjust ports and parameters in code if necessary).

2. **Launch the node**:
   ```bash
   ros2 run phi_p3dx_aria phi_p3dx
   ```

## License

This project is distributed under the GPLv2 license. See the [LICENSE](LICENSE) file for details.

The original code (ROSARIA) was GPLv2, and this ROS2 version maintains compatibility with the original license.
