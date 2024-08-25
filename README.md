# Go2 Gait Planner

## Overview

The **Go2 Gait Planner** is a project developed to implement a simple gait planning algorithm for the Unitree Go2 Quadruped Robot. This planner is designed to enable easy and friendly definition of gait patterns which is executed by underlying leg controllers. The project involves setting up a simulation environment using MuJoCo, developing kinematic of the robot, and validating these models through simulations and real-world tests.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Simulation Environment](#simulation-environment)
- [Real-World Implementation](#real-world-implementation)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Features

- **Gait Planning**: Pre-defined trot and jump gait patterns.
- **Simulation Environment**: Utilizes MuJoCo for simulating the robot's movement.
- **Kinematic Modeling**: Includes detailed models of the Unitree Go2 Quadruped Robot using Jacobian Numerical Inverse-Kinematics in estimating foot position trajectory.
- **Networked Control**: Establishes a network connection between the robot and the development environment for real-time control and testing.

## Installation

To get started with the Go2 Gait Planner, follow these steps:

### Prerequisites

- [ROS2 (Robot Operating System 2)](https://docs.ros.org/en/foxy/Installation.html)
- [Unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [Eigen 3.4.0](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- Git

### Clone the Repository

```bash
git clone https://github.com/felixokolo/go2_gait_planner.git
cd go2_gait_planner
colcon build --packages-select go2_gait_planner
