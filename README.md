# barracuda-control

## SetThrustZero Service

The `SetThrustZero` service provides emergency thrust control capability, allowing external systems to immediately override the LQR controller and set all thrusters to zero.

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