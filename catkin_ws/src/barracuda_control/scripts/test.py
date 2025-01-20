#!/usr/bin/env python3  

import rospy
from geometry_msgs.msg import Wrench

def publish_wrench():
    # Initialize the ROS node
    rospy.init_node('wrench_publisher', anonymous=True)
    
    # Create a publisher for the /cmd_wrench topic
    pub = rospy.Publisher('barracuda/thruster_manager/input', Wrench, queue_size=10)
    
    # Set the publish rate
    rate = rospy.Rate(10)  # 10 Hz

    # Create a Wrench message
    wrench_msg = Wrench()

    # Apply force along the x-axis to move forward
    wrench_msg.force.x = 0.0  # Adjust the value as needed
    wrench_msg.force.y = 0.0
    wrench_msg.force.z = 0.0

    # No torque applied
    wrench_msg.torque.x = 0.0  # Adjust the value as needed
    wrench_msg.torque.y = 0.0
    wrench_msg.torque.z = 20.0

    rospy.loginfo("Publishing Wrench message to move forward")

    while not rospy.is_shutdown():
        pub.publish(wrench_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_wrench()
    except rospy.ROSInterruptException:
        pass
