# SetThrustZero Service

This document explains how to use the `SetThrustZero` service to control thrust output from the LQR controller.

## Overview

The `SetThrustZero` service provides a way to immediately override the LQR controller's thrust output and set all thrusters to zero. This is useful for:
- Emergency stops
- Testing scenarios where no thrust is desired
- Safely disabling thrust while keeping the control system running

## Service Definition

**Service Name:** `set_thrust_zero`

**Request:**
- `enable_thrust_zero` (bool): Set to `true` to disable all thrust, `false` to resume normal operation

**Response:**
- `success` (bool): Always `true` if service call succeeds
- `message` (string): Informational message about the operation

## Usage Examples

### Command Line
```bash
# Disable all thrust (set to zero)
rosservice call /barracuda/set_thrust_zero "enable_thrust_zero: true"

# Resume normal LQR control
rosservice call /barracuda/set_thrust_zero "enable_thrust_zero: false"
```

### Python
```python
import rospy
from barracuda_control.srv import SetThrustZero

# Wait for service
rospy.wait_for_service('/barracuda/set_thrust_zero')
set_thrust_zero = rospy.ServiceProxy('/barracuda/set_thrust_zero', SetThrustZero)

# Disable thrust
response = set_thrust_zero(enable_thrust_zero=True)
print(f"Success: {response.success}, Message: {response.message}")

# Resume normal control
response = set_thrust_zero(enable_thrust_zero=False)
print(f"Success: {response.success}, Message: {response.message}")
```

## Behavior

- **When thrust zero is enabled:** The LQR node publishes zero force and torque values on all axes
- **When thrust zero is disabled:** The LQR node resumes publishing computed control values
- **LQR computation continues:** The control algorithm keeps running even when thrust is zeroed
- **State estimation continues:** All sensors and state updates continue normally

## Implementation Details

The service modifies the `publishControl()` method in the LQR node:
- If `thrust_zero_enabled` is true: publishes zero wrench
- If `thrust_zero_enabled` is false: publishes normal LQR-computed wrench

This ensures minimal impact on the overall control system while providing the requested override capability.