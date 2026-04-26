# phi_p3dx_description

This ROS2 package contains the complete description of the Pioneer P3-DX robot, including URDF/Xacro models, Gazebo simulation configurations, MobileSim maps, and RViz configuration files.

## Overview

The `phi_p3dx_description` package provides:
- **Robot URDF/Xacro model**: Complete description of the Pioneer P3-DX with chassis, wheels, sensors (laser, camera, sonar).
- **Gazebo models**: SDF files for physical simulation.
- **Simulation worlds**: Empty scenarios and with obstacles.
- **RViz configurations**: Pre-configured visualizations for monitoring.
- **Maps**: Map files for navigation in the MobileSim simulator.

### Package Structure
- `urdf/`: Robot URDF/Xacro files (chassis, wheels, sensors).
- `models/`: Gazebo models (robot, camera, laser, obstacles).
- `worlds/`: SDF worlds for Gazebo.
- `rviz/`: RViz configurations.
- `map/`: Maps for navigation.

## Prerequisites

- **ROS2 Humble** (or compatible).
- **Gazebo** (for simulation).
- **MobileSim** (for simulation).
- Dependencies: `urdf`, `xacro`, `robot_state_publisher`, `joint_state_publisher`.

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/phir2-lab/phi_p3dx_description.git
   ```

2. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select phi_p3dx_description
   source install/setup.bash
   ```
## License

This project is distributed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.