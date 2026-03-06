#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Vector3, Wrench
from rclpy.node import Node
from sensor_msgs.msg import Joy

MAX_FORCE = 4
MAX_TORQUE = 4


class JoystickToWrench(Node):
    def __init__(self):
        super().__init__("joystick_to_wrench")
        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.joy_subscriber_callback, 10
        )
        self.joy_subscription  # prevent unused variable warning

        self.wrench_publisher = self.create_publisher(Wrench, "wrench", 10)

    def joy_subscriber_callback(self, msg):
        if not msg.axes or not msg.buttons:
            self.get_logger().warning("malformatted joy message")
            return

        wrench = Wrench()
        force = (
            msg.axes[1],
            -msg.axes[0],
            1.0 if msg.buttons[12] == 1 else (-1.0 if msg.buttons[13] == 1 else 0.0),
        )
        wrench.force = Vector3()
        for axis, value in zip(("x", "y", "z"), force):
            setattr(wrench.force, axis, value * MAX_FORCE)

        torque = [
            -msg.axes[2],
            msg.axes[3],
            -1.0 if msg.buttons[4] == 1 else (1.0 if msg.buttons[5] == 1 else 0.0),
        ]
        wrench.torque = Vector3()
        for axis, value in zip(("x", "y", "z"), torque):
            setattr(wrench.torque, axis, value * MAX_TORQUE)

        self.wrench_publisher.publish(wrench)


def main(args=None):
    rclpy.init(args=args)

    joystick_to_wrench = JoystickToWrench()

    rclpy.spin(joystick_to_wrench)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_to_wrench.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
