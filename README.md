# UR10e Vertical 7th Axis Ignition Gazebo Simulation with MoveIt

This repository contains a comprehensive ROS2 simulation of a UR10e robotic arm from Universal Robotics, mounted on a vertical platform with a prismatic joint that enables movement along the z-axis from 0.0 to 1.0 meters. The simulation is built using Ignition Gazebo and integrated with MoveIt2 for advanced motion planning and control.

## Features

- **UR10e Robotic Arm**: Full simulation of the 6-DOF UR10e manipulator
- **7th Axis Motion**: Vertical platform with prismatic joint for extended workspace
- **Ignition Gazebo Integration**: High-fidelity physics simulation environment
- **MoveIt2 Support**: Motion planning, collision detection, and trajectory execution
- **ROS2 Control**: Real-time control interface with ros2_control framework

## Prerequisites

Before installing this package, ensure you have the following dependencies:

- **ROS2 Humble** (Desktop Full installation recommended)
- **Ignition Gazebo** (Fortress)
- **MoveIt2** for ROS2 Humble
- **ros2_control** and related packages
- **colcon** build tool

## Installation and Setup

Follow these steps to set up the simulation in your local environment:

### 1. Create a ROS2 Workspace

```bash
mkdir -p ~/ur10e_ws/src
```

### 2. Navigate to Workspace Directory

```bash
cd ~/ur10e_ws
```

### 3. Initial Workspace Build

```bash
colcon build --symlink-install
```

### 4. Navigate to Source Directory

```bash
cd src
```

### 5. Clone the Repository

```bash
git clone https://github.com/G-Paul/ur10e-7th-axis.git
```

### 6. Return to Workspace Root

```bash
cd ..
```

### 7. Initialize and Update rosdep

```bash
# Initialize rosdep (skip if already initialized)
sudo rosdep init

# Update rosdep database
rosdep update
```

### 8. Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 9. Build the Workspace

```bash
colcon build --symlink-install
```

### 10. Source the Workspace

```bash
source install/setup.bash
```

## Usage

### Launching the Simulation

```bash
# Launch Ignition Gazebo with the UR10e and 7th axis and MoveIt:
ros2 launch construction_bot lift_ur_moveit.launch.py world_file:=src/ur10e-7th-axis/construction_bot/world/no_gravity.sdf use_sim_time:=true

# Launch only the rviz visualizaiton of the robot [with joint-state-publisher-gui]:
ros2 launch construction_bot view_ur.launch.py
```

## Acknowledgments

- Universal Robotics for the UR10e specifications
- ROS2 and MoveIt2 communities
- Ignition Gazebo development team
