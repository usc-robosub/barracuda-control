from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, FindPackageShare, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare as RosFindPackageShare


def generate_launch_description():
    # Declare arguments
    reset_tam_arg = DeclareLaunchArgument(
        'reset_tam',
        default_value='true',
        description='Whether to reset TAM'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=PathJoinSubstitution([FindPackageShare('barracuda_control'), 'config']),
        description='Output directory for TAM.yaml'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([FindPackageShare('barracuda_control'), 'config', 'thruster_manager.yaml']),
        description='Thruster manager config file'
    )
    
    tam_file_arg = DeclareLaunchArgument(
        'tam_file',
        default_value=PathJoinSubstitution([FindPackageShare('barracuda_control'), 'config', 'TAM.yaml']),
        description='TAM file path'
    )

    # Include thruster manager launch file
    thruster_manager_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('barracuda_control'), 'launch', 'thruster_manager.py'])
    )

    # LQR node
    lqr_node = Node(
        package='barracuda_control',
        executable='lqr_node',
        namespace='barracuda',
        parameters=[
            PathJoinSubstitution([FindPackageShare('barracuda_control'), 'config', 'lqr_params.yaml'])
        ],
        output='screen'
    )

    return LaunchDescription([
        reset_tam_arg,
        output_dir_arg,
        config_file_arg,
        tam_file_arg,
        thruster_manager_launch,
        lqr_node,
    ])
