# phi_p3dx_navigation

This repository contains ROS2 packages for autonomous navigation of the Pioneer P3-DX robot in different environments: Gazebo simulation (3D), MobileSim (2D), and real robot.

## Overview

The `phi_p3dx_navigation` package provides:
- **Reactive navigation nodes**: Simple control implementations for obstacle avoidance and goal reaching.
- **Multi-platform compatibility**: Works in Gazebo, MobileSim, and physical robots.
- **Educational examples**: Commented code in Python and C++ for robotics students.

### Package Structure
- `scripts/`: Python scripts (e.g., `control_example.py`).
- `phi_p3dx_navigation/`: Python base class
- `src/`: C++ code (e.g., `control_example.cpp`, `navigation_node.cpp`).
- `include/`: C++ headers (e.g., `navigation_node.hpp`).
- `launch/`: Launch files for different environments.
- `config/`: Configurations (e.g., RViz, maps).

## Prerequisites

- **ROS2 Humble** (or compatible).
- **Gazebo** (for 3D simulation).
- **MobileSim** (for 2D simulation, if available).
- Dependencies: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2`.

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/phir2-lab/phi_p3dx_navigation.git
   ```

2. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select phi_p3dx_navigation
   source install/setup.bash
   ```

## Usage

### Gazebo Simulation (3D)

1. Launch the simulation:
   ```bash
   ros2 launch phi_p3dx_navigation bringup_gazebo.launch.py
   ```
2. Run the navigation node (Python):
   ```bash
   ros2 run phi_p3dx_navigation control_example_py
   ```

   Or in C++:
   ```bash
   ros2 run phi_p3dx_navigation control_example_cpp
   ```

3. In RViz:
   - Use "2D Nav Goal" to send goals.


### MobileSim Simulation (2D)

1. Launch the simulation:
   ```bash
   ros2 launch phi_p3dx_navigation bringup_mobilesim.launch.py
   ```

2. Run the navigation node as above.

### Real Robot

1. Connect the Pioneer P3-DX robot.

2. Launch the system:
   ```bash
   ros2 launch phi_p3dx_navigation bringup_real.launch.py
   ```

3. Run the navigation node.

## Navigation Examples

### Basic Algorithm
- **Turn towards goal**: Calculate angle error and rotate.
- **Go straight**: Advance when aligned.
- **Avoid obstacles**: Stop if something is detected ahead (threshold: 0.5m).
- **Goal reached**: Stop when close enough (threshold: 0.1m).

## Documentation

- **Python**: Docstrings can be viewed with `pydoc` or Sphinx.
- **C++**: Doxygen comments. Generate docs with:
  ```bash
  cd src/phi_p3dx_navigation
  doxygen -g  # Create Doxyfile
  doxygen     # Generate docs in html/
  ```

## Alternative Branches

- **`mobilesim-only`**: A version without Gazebo support, focused only on MobileSim (2D) and real robot navigation. This branch removes Gazebo dependencies and launch files for users who prefer to stick with ROS2 Humble's default Gazebo Classic or don't need 3D simulation.

  To use this branch:
  ```bash
  git checkout mobilesim-only
  ```

## License

This project is distributed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
