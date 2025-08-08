# Barracuda Control

Barracuda's control package, including an LQR, thruster manager, and thruster override service.

## Overview

Barracuda Control implements a sophisticated 6-DOF optimal control system using modern control theory and high-performance optimization libraries. The system provides autonomous navigation capabilities with built-in safety features and emergency controls.

### Key Features

- **Optimal Control**: LQR-based controller for 15-dimensional state space
- **High Performance**: 50 Hz control loop with optimized linear algebra
- **Safety First**: Emergency thrust shutdown service
- **Modular Design**: Clean separation between control logic and thruster allocation
- **Docker Ready**: Containerized deployment with automated builds
- **ROS Integration**: Standard ROS interfaces for seamless integration

## System Architecture

### Control System
- **State Representation** (15 dimensions):
  - Position: `[x, y, z]` (3D coordinates)
  - Orientation: `[roll, pitch, yaw]` (Euler angles)
  - Linear velocity: `[u, v, w]` (body frame velocities)
  - Angular velocity: `[p, q, r]` (body frame angular rates)
  - Linear acceleration: `[ax, ay, az]` (body frame accelerations)

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
git clone https://github.com/your-org/barracuda-control.git
cd barracuda-control

# Build and run with Docker Compose
docker-compose up --build
```

### Manual Installation

```bash
# Install ROS Noetic and dependencies
sudo apt-get install ros-noetic-ros-base libeigen3-dev libboost-all-dev liblapack-dev python3-catkin-tools

# Clone and build dependencies
cd dependencies/blasfeo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
make -j4 && sudo make install

cd ../../hpipm && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
make -j4 && sudo make install

# Build the ROS workspace
cd ../../catkin_ws
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
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
  - `/odometry/filtered` (nav_msgs/Odometry): Vehicle pose and velocity
  - `/accel/filtered` (geometry_msgs/AccelStamped): Vehicle acceleration
  - `/cmd_vel` (geometry_msgs/Twist): Velocity setpoint commands

- **Published**:
  - `/thruster_manager/input` (uuv_gazebo_ros_plugins_msgs/FloatStamped): Force/torque commands

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

### Usage Examples

**Command Line:**
```bash
# Emergency stop - disable all thrust
rosservice call /barracuda/set_thrust_zero "enable_thrust_zero: true"

# Resume normal control
rosservice call /barracuda/set_thrust_zero "enable_thrust_zero: false"
```

**Python:**
```python
import rospy
from barracuda_control.srv import SetThrustZero

# Wait for service and create proxy
rospy.wait_for_service('/barracuda/set_thrust_zero')
set_thrust_zero = rospy.ServiceProxy('/barracuda/set_thrust_zero', SetThrustZero)

# Emergency stop
response = set_thrust_zero(enable_thrust_zero=True)
print(f"Success: {response.success}, Message: {response.message}")

# Resume normal operation
response = set_thrust_zero(enable_thrust_zero=False)
```

**C++:**
```cpp
#include <ros/ros.h>
#include <barracuda_control/SetThrustZero.h>

// Create service client
ros::ServiceClient client = nh.serviceClient<barracuda_control::SetThrustZero>("/barracuda/set_thrust_zero");

// Emergency stop
barracuda_control::SetThrustZero srv;
srv.request.enable_thrust_zero = true;
if (client.call(srv)) {
    ROS_INFO("Thrust disabled: %s", srv.response.message.c_str());
}
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

### Contributing
1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes and test thoroughly
4. Submit a pull request

## UUV Simulator Thruster Manager

The system integrates UUV Simulator's thruster manager (`thruster_manager.py:31-384`) for thruster allocation and control. It automatically generates the Thruster Allocation Matrix (TAM) using TF frame information, converts 6-DOF force/torque commands to individual thruster forces, and handles coordinate frame transformations with thrust limiting.

The LQR controller publishes commands to `/thruster_manager/input`, which are converted to individual thruster commands using the computed allocation matrix.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- **Control Toolbox (CT)**: ETH Zurich's Automatic Control Laboratory
- **UUV Simulator**: Laboratory for Underwater Systems and Technologies (LSTS)
- **BLASFEO/HPIPM**: University of Freiburg's Systems Control and Optimization Laboratory
