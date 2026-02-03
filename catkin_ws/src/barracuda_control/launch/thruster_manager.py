from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindPackageShare, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare as RosFindPackageShare
import os


def generate_launch_description():
    # Declare all arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        description='Name of the model'
    )
    
    uuv_name_arg = DeclareLaunchArgument(
        'uuv_name',
        default_value=LaunchConfiguration('model_name'),
        description='Name of the UUV'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Base link frame'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='-1',
        description='Thruster manager timeout'
    )
    
    reset_tam_arg = DeclareLaunchArgument(
        'reset_tam',
        default_value='false',
        description='Reset TAM'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=[FindPackageShare('uuv_thruster_manager'), '/config/'],
        description='Output directory'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=[FindPackageShare('uuv_thruster_manager'), '/config/thruster_manager.yaml'],
        description='Config file path'
    )
    
    tam_file_arg = DeclareLaunchArgument(
        'tam_file',
        default_value=[FindPackageShare('uuv_thruster_manager'), '/config/TAM.yaml'],
        description='TAM file path'
    )

    # Thruster allocator node
    thruster_allocator = Node(
        package='thruster_manager',
        executable='thruster_manager_node',
        name='thruster_manager',
        namespace='barracuda',
        parameters=[
            {'tam.min_thrust': 0.0},
            {'tam.max_thrust': 100.0},
        ]
    )

    return LaunchDescription([
        model_name_arg,
        uuv_name_arg,
        base_link_arg,
        timeout_arg,
        reset_tam_arg,
        output_dir_arg,
        config_file_arg,
        tam_file_arg,
        thruster_allocator,
    ])
