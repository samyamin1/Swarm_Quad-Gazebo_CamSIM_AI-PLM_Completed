#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('quadcopter_controller')
    
    # Launch arguments
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([pkg_dir, 'worlds', 'empty_world.world']),
        description='Path to world file'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Set environment variables
    os.environ['GAZEBO_MODEL_PATH'] = PathJoinSubstitution([pkg_dir, 'models'])
    os.environ['GAZEBO_RESOURCE_PATH'] = PathJoinSubstitution([pkg_dir, 'worlds'])
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )
    
    # Spawn quadcopter
    spawn_quadcopter = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_quadcopter',
        arguments=[
            '-entity', 'quadcopter',
            '-file', PathJoinSubstitution([pkg_dir, 'models', 'quadcopter', 'model.sdf']),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '2.0'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # RViz for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_dir, 'config', 'quadcopter.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_world_file,
        declare_use_sim_time,
        gazebo,
        spawn_quadcopter,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ]) 