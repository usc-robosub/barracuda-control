#!/usr/bin/env python3

import rospy
from barracuda_control.srv import SetThrustZero, SetThrustZeroRequest

def test_thrust_zero_service():
    """Test script to demonstrate the SetThrustZero service"""
    rospy.init_node('test_thrust_zero', anonymous=True)
    
    # Wait for the service to be available
    rospy.wait_for_service('/barracuda/set_thrust_zero')
    
    try:
        # Create service proxy
        set_thrust_zero = rospy.ServiceProxy('/barracuda/set_thrust_zero', SetThrustZero)
        
        # Test enabling thrust zero
        rospy.loginfo("Setting thrust to zero...")
        req = SetThrustZeroRequest()
        req.enable_thrust_zero = True
        response = set_thrust_zero(req)
        rospy.loginfo(f"Response: success={response.success}, message='{response.message}'")
        
        # Wait a few seconds
        rospy.sleep(3.0)
        
        # Test disabling thrust zero (resume normal control)
        rospy.loginfo("Resuming normal thrust control...")
        req.enable_thrust_zero = False
        response = set_thrust_zero(req)
        rospy.loginfo(f"Response: success={response.success}, message='{response.message}'")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        test_thrust_zero_service()
    except rospy.ROSInterruptException:
        pass