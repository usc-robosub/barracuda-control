#!/usr/bin/env python3

"""Publish a configurable target pose for the LQR node."""

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


def publish_target_pose():
    """Publish the target pose from parameters to ``target_odometry``."""
    rospy.init_node("test_target_pose", anonymous=True)

    position = rospy.get_param("target_pose/position", {})
    orientation = rospy.get_param("target_pose/orientation", {})

    odom_pub = rospy.Publisher("target_odometry", Odometry, queue_size=10)
    rate = rospy.Rate(10)

    odom_msg = Odometry()
    odom_msg.header.frame_id = "map"
    odom_msg.pose.pose.position.x = position.get("x", 0.0)
    odom_msg.pose.pose.position.y = position.get("y", 0.0)
    odom_msg.pose.pose.position.z = position.get("z", 0.0)

    q = quaternion_from_euler(
        orientation.get("roll", 0.0),
        orientation.get("pitch", 0.0),
        orientation.get("yaw", 0.0),
    )
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]

    while not rospy.is_shutdown():
        odom_msg.header.stamp = rospy.Time.now()
        odom_pub.publish(odom_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_target_pose()
    except rospy.ROSInterruptException:
        pass

