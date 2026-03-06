from launch import LaunchDescription
from launch_ros.actions import Node

node_joystick_to_wrench = Node(
    package="barracuda_control",
    namespace="barracuda",
    executable="joystick_to_wrench.py",
    output="screen",
)


def generate_launch_description():
    return LaunchDescription(
        [
            node_joystick_to_wrench,
        ]
    )
