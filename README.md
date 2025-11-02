# TurtleBot3 Workspace with Custom DWA Local Planner

A ROS 2 workspace for TurtleBot3 simulation and navigation with a custom Python implementation of the Dynamic Window Approach (DWA) local planner.

## 📋 Table of Contents
- [Overview](#overview)
- [Demo](#demo)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Workspace Structure](#workspace-structure)
- [Usage](#usage)
- [Custom DWA Local Planner](#custom-dwa-local-planner)
- [TurtleBot3 Simulations](#turtlebot3-simulations)
- [Contributing](#contributing)
- [License](#license)

## 🔍 Overview

This workspace contains:
- **Custom DWA Local Planner**: A Python-based implementation of the Dynamic Window Approach algorithm for local path planning
- **TurtleBot3 Simulations**: Official TurtleBot3 simulation packages including Gazebo environments, fake nodes, and manipulation support

## 🎬 Demo

### Navigation with Custom DWA Planner

<div align="center">

https://github.com/LLawliet2028/turtlebot_ws/docs/videos/demo.mp4

*TurtleBot3 navigating through obstacles using custom DWA local planner*

</div>

> **Note:** If the video doesn't display above, you can view it directly [here](docs/videos/demo.mp4)

### Simulation Environments

<div align="center">
  <img src="docs/images/rviz.jpeg" alt="RViz Laser Scan Visualization" width="700"/>
  <br />
  <em>RViz Visualization with Laser Scan Data and TurtleBot3</em>
</div>

<br />

> **Note:** More simulation environment screenshots will be added soon.

### Obstacle Avoidance Demo

<div align="center">

> **Video:** [View Obstacle Avoidance Demo](docs/videos/demo.mp4)

*Real-time obstacle avoidance with dynamic obstacles*

</div>

### Path Planning in Complex Environment

<div align="center">

<img src="docs/images/navigation_demo.gif" alt="Navigation Demo" width="600"/>

*Autonomous navigation from start to goal position*

</div>

## ✨ Features

- 🤖 Complete TurtleBot3 simulation environment
- 🎯 Custom DWA (Dynamic Window Approach) local planner implementation
- 🌍 Multiple Gazebo worlds for testing and development
- 🦾 TurtleBot3 manipulation support with Gazebo
- 📊 RViz configurations for visualization
- 🔧 Flexible and extensible architecture

## 📦 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 (Jammy) recommended
- **ROS 2**: Humble Hawksbill or later
- **Python**: 3.10+

### Dependencies
```bash
# ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# Python dependencies
pip3 install numpy
```

### Environment Variables
```bash
# Add to ~/.bashrc
export TURTLEBOT3_MODEL=burger  # or waffle, waffle_pi
source /opt/ros/humble/setup.bash
```

## 🚀 Installation

1. **Clone the repository**
```bash
git clone https://github.com/LLawliet2028/turtlebot_ws.git
cd turtlebot_ws
```

2. **Install dependencies**
```bash
cd src/turtlebot3_simulations
rosdep install -i --from-path . --rosdistro humble -y
cd ../..
```

3. **Build the workspace**
```bash
colcon build --symlink-install
```

4. **Source the workspace**
```bash
source install/setup.bash
```

## 📁 Workspace Structure

```
turtlebot_ws/
├── src/
│   ├── dwa_local_planner/          # Custom DWA implementation
│   │   ├── dwa_local_planner/
│   │   │   ├── __init__.py
│   │   │   └── dwa_planner.py     # Main DWA algorithm
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│   │
│   └── turtlebot3_simulations/     # TurtleBot3 simulation packages
│       ├── turtlebot3_fake_node/   # Simulated robot without Gazebo
│       ├── turtlebot3_gazebo/      # Gazebo simulation environments
│       ├── turtlebot3_manipulation_gazebo/  # Manipulation simulations
│       └── turtlebot3_simulations/ # Meta-package
```

## 🎮 Usage

### Basic Gazebo Simulation

**Launch TurtleBot3 in an empty world:**
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Launch TurtleBot3 in TurtleBot3 World:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Launch TurtleBot3 in House environment:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Teleoperation

**Control the robot with keyboard:**
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Using the Custom DWA Local Planner

**Run the DWA planner node:**
```bash
ros2 run dwa_local_planner dwa_planner
```

The DWA planner subscribes to:
- `/odom` - Robot odometry
- `/scan` - Laser scan data
- `/goal_pose` - Target goal position

And publishes to:
- `/cmd_vel` - Velocity commands

### Navigation with Nav2

**Launch navigation stack with custom DWA planner:**
```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch Navigation
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3: Launch RViz
ros2 launch nav2_bringup rviz_launch.py
```

### Manipulation Simulation

**Launch TurtleBot3 with OpenManipulator:**
```bash
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_manipulation.launch.py
```

## 🎯 Custom DWA Local Planner

The Dynamic Window Approach (DWA) is a velocity-based local planner that:
- Generates velocity samples within the robot's dynamic constraints
- Evaluates trajectories based on goal direction, obstacle clearance, and velocity
- Selects optimal velocity commands in real-time

### Key Features
- Python-based implementation for easy modification
- Configurable parameters for different robot behaviors
- Real-time obstacle avoidance
- Smooth trajectory generation

### Algorithm Parameters (configurable in `dwa_planner.py`)
- `max_speed`: Maximum linear velocity
- `min_speed`: Minimum linear velocity
- `max_yaw_rate`: Maximum angular velocity
- `max_accel`: Maximum linear acceleration
- `max_delta_yaw_rate`: Maximum angular acceleration
- `velocity_resolution`: Velocity sampling resolution
- `dt`: Time step for trajectory prediction
- `predict_time`: Trajectory prediction horizon
- `to_goal_cost_gain`: Weight for goal direction
- `speed_cost_gain`: Weight for velocity preference
- `obstacle_cost_gain`: Weight for obstacle avoidance

### Customization Example
```python
# Modify parameters in dwa_planner.py
self.max_speed = 0.5  # m/s
self.max_yaw_rate = 1.0  # rad/s
self.obstacle_cost_gain = 2.0  # Increase obstacle avoidance
```

## 🌍 Available Gazebo Worlds

1. **Empty World** - Basic testing environment
2. **TurtleBot3 World** - Obstacles and structures
3. **TurtleBot3 House** - Indoor home environment
4. **Custom Worlds** - Located in `turtlebot3_gazebo/worlds/`

## 🔧 Configuration

### Robot Model Selection
```bash
export TURTLEBOT3_MODEL=burger  # Options: burger, waffle, waffle_pi
```

### RViz Configuration
RViz configuration files are available in:
- `turtlebot3_gazebo/rviz/`
- `turtlebot3_manipulation_gazebo/rviz/`

## 🐛 Troubleshooting

### Gazebo Models Not Found
```bash
# Set Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
```

### DWA Planner Issues
- Ensure odometry and laser scan topics are publishing
- Check parameter tuning in `dwa_planner.py`
- Verify coordinate frame transformations

### Build Errors
```bash
# Clean build
rm -rf build install log
colcon build --symlink-install
```

## 📚 Resources

- [TurtleBot3 Official Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS 2 Navigation2 Documentation](https://navigation.ros.org/)
- [DWA Algorithm Paper](https://ieeexplore.ieee.org/document/580977)
- [Gazebo Tutorials](https://gazebosim.org/docs)

## 🤝 Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](src/turtlebot3_simulations/CONTRIBUTING.md) for guidelines.

### Development Workflow
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project includes:
- Custom DWA Local Planner: [Specify your license]
- TurtleBot3 Simulations: Apache License 2.0 (see [LICENSE](src/turtlebot3_simulations/LICENSE))

## 👤 Author

**LLawliet2028**
- GitHub: [@LLawliet2028](https://github.com/LLawliet2028)

## 🙏 Acknowledgments

- ROBOTIS for TurtleBot3 platform and simulation packages
- ROS 2 and Navigation2 communities
- DWA algorithm original authors

## 📝 Citation

If you use this workspace in your research, please cite:
```bibtex
@software{turtlebot_ws_2024,
  author = {LLawliet2028},
  title = {TurtleBot3 Workspace with Custom DWA Local Planner},
  year = {2024},
  url = {https://github.com/LLawliet2028/turtlebot_ws}
}
```

---

**Happy Robot Navigation! 🤖🚀**

For questions or issues, please open an issue on GitHub.
