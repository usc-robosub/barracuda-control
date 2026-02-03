from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Load LQR parameters and start test target pose publisher
    test_target_pose = Node(
        package='barracuda_control',
        executable='test_target_pose.py',
        name='test_target_pose',
        parameters=[
            PathJoinSubstitution([FindPackageShare('barracuda_control'), 'config', 'lqr_params.yaml'])
        ],
        output='screen'
    )

    return LaunchDescription([
        test_target_pose,
    ])
