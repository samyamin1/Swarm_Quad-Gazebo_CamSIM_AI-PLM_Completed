#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Flight controller node
    flight_controller = Node(
        package='quadcopter_controller',
        executable='flight_controller_node.py',
        name='flight_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'control_rate': 100.0,
            'max_velocity': 10.0,
            'max_angular_velocity': 2.0,
            'max_altitude': 100.0
        }]
    )
    
    # Teleop keyboard node
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/quadcopter/control/velocity')
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        flight_controller,
        teleop_keyboard
    ]) 