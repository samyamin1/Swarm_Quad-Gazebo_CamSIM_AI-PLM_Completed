#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('full_system_integration')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='office_building.world')
    enable_gazebo_gui = LaunchConfiguration('enable_gazebo_gui', default='true')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='office_building.world',
        description='Gazebo world file to load'
    )
    
    declare_enable_gazebo_gui = DeclareLaunchArgument(
        'enable_gazebo_gui',
        default_value='true',
        description='Enable Gazebo GUI'
    )
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    # Stage 1: Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                pkg_share, 'worlds', world_file
            ]),
            'gui': enable_gazebo_gui,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Stage 1: Quadcopter Controller
    quadcopter_controller = Node(
        package='quadcopter_controller',
        executable='flight_controller',
        name='quadcopter_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'quadcopter_id': 'quadcopter_1',
            'max_velocity': 5.0,
            'max_acceleration': 2.0,
        }]
    )
    
    # Stage 2: 3D Map Creation (already loaded in Gazebo)
    
    # Stage 3: Sensor Simulation
    camera_simulator = Node(
        package='sensor_simulator',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_topic': '/quadcopter/camera/image_raw',
            'frame_rate': 30.0,
        }]
    )
    
    lidar_simulator = Node(
        package='sensor_simulator',
        executable='lidar_simulator',
        name='lidar_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'lidar_topic': '/quadcopter/lidar/scan',
            'scan_rate': 10.0,
        }]
    )
    
    # Stage 4: SLAM Engine
    slam_engine = Node(
        package='slam_engine',
        executable='visual_slam',
        name='slam_engine',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': '/quadcopter/slam/map',
            'pose_topic': '/quadcopter/slam/pose',
        }]
    )
    
    # Stage 5: Perception Language Model
    perception_plm = Node(
        package='perception_plm',
        executable='vision_plm',
        name='perception_plm',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'confidence_threshold': 0.7,
            'update_rate': 5.0,
        }]
    )
    
    # Stage 6: LLM Decision Engine
    llm_decision_engine = Node(
        package='llm_decision_engine',
        executable='decision_engine',
        name='llm_decision_engine',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'decision_frequency': 2.0,
            'mission_type': 'exploration',
        }]
    )
    
    # Stage 7: Navigation Control Loop
    navigation_controller = Node(
        package='navigation_control',
        executable='navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'control_frequency': 20.0,
            'max_velocity': 3.0,
        }]
    )
    
    # Stage 8: Swarm Coordination
    swarm_coordinator = Node(
        package='swarm_coordination',
        executable='swarm_coordinator',
        name='swarm_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'formation_type': 'triangle',
            'separation_distance': 2.0,
        }]
    )
    
    # Stage 9: Multi-Sensor Fusion
    sensor_fusion = Node(
        package='sensor_fusion',
        executable='sensor_fusion_engine',
        name='sensor_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fusion_rate': 10.0,
            'filter_type': 'kalman',
        }]
    )
    
    # Stage 10: Advanced SLAM
    advanced_slam = Node(
        package='advanced_slam',
        executable='advanced_slam_engine',
        name='advanced_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'slam_type': 'hybrid',
            'update_rate': 5.0,
        }]
    )
    
    # Stage 11: Real-time AI Integration
    realtime_ai_integration = Node(
        package='realtime_ai',
        executable='realtime_ai_integration',
        name='realtime_ai_integration',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'integration_frequency': 20.0,
            'component_timeout': 2.0,
        }]
    )
    
    # Stage 12: Mission Planning & Execution
    mission_planner = Node(
        package='mission_planning',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planning_frequency': 5.0,
            'mission_type': 'exploration',
        }]
    )
    
    # Stage 13: Performance Optimization
    performance_optimizer = Node(
        package='performance_optimizer',
        executable='performance_optimizer',
        name='performance_optimizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitoring_frequency': 10.0,
            'optimization_threshold': 0.8,
        }]
    )
    
    # Stage 14: Full System Integration (Main Controller)
    system_integrator = Node(
        package='full_system_integration',
        executable='system_integrator',
        name='system_integrator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'integration_frequency': 5.0,
            'heartbeat_timeout': 10.0,
        }]
    )
    
    # RViz for visualization
    rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'swarm_system.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # System health monitoring
    system_monitor = Node(
        package='full_system_integration',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitoring_rate': 1.0,
        }]
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_world_file,
        declare_enable_gazebo_gui,
        declare_enable_rviz,
        
        # Start Gazebo simulation
        gazebo_launch,
        
        # Start all system components
        quadcopter_controller,
        camera_simulator,
        lidar_simulator,
        slam_engine,
        perception_plm,
        llm_decision_engine,
        navigation_controller,
        swarm_coordinator,
        sensor_fusion,
        advanced_slam,
        realtime_ai_integration,
        mission_planner,
        performance_optimizer,
        system_integrator,
        
        # Visualization and monitoring
        rviz_node,
        system_monitor,
    ]) 