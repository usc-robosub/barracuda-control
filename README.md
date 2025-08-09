# Barracuda Control

Barracuda's control package, including an LQR, thruster manager, and thruster override service.

## Overview

Barracuda Control implements a sophisticated 6-DOF optimal control system using modern control theory and high-performance optimization libraries. The system provides autonomous navigation capabilities with built-in safety features and emergency controls.

### Key Features

- **Optimal Control**: LQR-based controller for a 12-dimensional state space
- **High Performance**: 50 Hz control loop with optimized linear algebra
- **Safety First**: Emergency thrust shutdown service
- **Modular Design**: Clean separation between control logic and thruster allocation
- **Docker Ready**: Containerized deployment with automated builds
- **ROS Integration**: Standard ROS interfaces for seamless integration

## System Architecture

### Control System
- **State Representation** (12 dimensions):
  - Position: `[x, y, z]` (3D coordinates)
  - Orientation error: quaternion vector part `[q_x, q_y, q_z]`
  - Linear velocity: `[u, v, w]` (body-frame velocities)
  - Angular velocity: `[p, q, r]` (body-frame angular rates)

- **Control Output**: 6D force/torque commands `[Fx, Fy, Fz, Mx, My, Mz]`
- **Update Rate**: 50 Hz discrete-time control loop

### Core Components

1. **LQR Controller Node** (`catkin_ws/src/barracuda_control/src/lqr_node.cpp`)
   - Implements discrete-time LQR optimal control
   - Processes odometry and acceleration feedback
   - Publishes force/torque commands to thruster manager

2. **Configuration System**
   - LQR tuning parameters (Q/R matrices)
   - Thruster allocation configuration
   - Launch file orchestration

3. **Emergency Safety Services**
   - SetThrustZero service for immediate thrust shutdown
   - Non-intrusive operation (control computation continues)

## Dependencies

### Core Control Libraries
- **Control Toolbox (CT)**: ETH Zurich's high-performance C++ control library
- **BLASFEO**: Optimized linear algebra routines
- **HPIPM**: Interior point method QP solver
- **Eigen3**: C++ linear algebra library

### ROS Dependencies
- **UUV Simulator**: Underwater vehicle simulation framework including thruster manager
- Standard ROS packages: `roscpp`, `geometry_msgs`, `nav_msgs`

## Installation

### Docker (Recommended)

```bash
# Clone the repository
git clone https://github.com/usc-robosub/barracuda-control.git
cd barracuda-control

# Build and run with Docker Compose
docker-compose up --build
```


## Usage

### Quick Start

```bash
# Launch the complete control system
roslaunch barracuda_control start_thruster_manager.launch
```

### Configuration

#### LQR Parameters (`config/lqr_params.yaml`)
```yaml
lqr:
  update_rate: 50.0  # Control loop frequency (Hz)
  Q: [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]  # State weights
  R: [1, 1, 1, 1, 1, 1]  # Control effort weights
```

#### Thruster Configuration (`config/thruster_manager.yaml`)
```yaml
thruster_manager:
  max_thrust: 6.0
  timeout: -1
  conversion_fcn: proportional
  conversion_fcn_params: {gain: 1.0}
```

### ROS Interface

#### Topics
- **Subscribed**:
  - `/odometry/filtered/global` (nav_msgs/Odometry): Vehicle pose and velocity
  - `/target_pose` (geometry_msgs/Twist): Velocity setpoint commands

- **Published**:
  - `/thruster_manager/input` (geometry_msgs/WrenchStamped): Force/torque commands

#### Services
- `/barracuda/set_thrust_zero` (barracuda_control/SetThrustZero): Emergency thrust control

## SetThrustZero Service

Emergency thrust control capability for immediate safety intervention.

### Service Definition

**Service Name:** `/barracuda/set_thrust_zero`  
**Service Type:** `barracuda_control/SetThrustZero`

```
# Request
bool enable_thrust_zero  # true to set thrust to zero, false to resume normal operation

---
# Response
bool success             # true if the operation succeeded
string message           # informational message
```

### Behavior

- **Emergency Stop Mode:** When `enable_thrust_zero` is `true`, all thrust output is set to zero
- **Normal Operation:** When `enable_thrust_zero` is `false`, normal LQR control resumes
- **Non-Intrusive:** LQR computation and state estimation continue running regardless of thrust override state

## Testing

### Automated Tests
```bash
# Run SetThrustZero service tests
cd catkin_ws/src/barracuda_control/scripts
python3 test_thrust_zero.py
./test_thrust_zero.sh
```

### Manual Testing
```bash
# Test thruster allocation without LQR
python3 test.py
```

## Development

### Building from Source
```bash
# Navigate to workspace
cd catkin_ws

# Build with debug symbols
catkin build -DCMAKE_BUILD_TYPE=Debug

# Build specific package
catkin build barracuda_control
```

## UUV Simulator Thruster Manager

The system integrates UUV Simulator's thruster manager (`thruster_manager.py:31-384`) for thruster allocation and control. It automatically generates the Thruster Allocation Matrix (TAM) using TF frame information, converts 6-DOF force/torque commands to individual thruster forces, and handles coordinate frame transformations with thrust limiting.

The LQR controller publishes commands to `/thruster_manager/input`, which are converted to individual thruster commands using the computed allocation matrix.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- **Control Toolbox (CT)**: ETH Zurich's Automatic Control Laboratory
- **UUV Simulator**: Laboratory for Underwater Systems and Technologies (LSTS)
- **BLASFEO/HPIPM**: University of Freiburg's Systems Control and Optimization Laboratory
