#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool, Int32
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2
import time
import json
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import math

class MissionType(Enum):
    SEARCH_AND_RESCUE = "search_and_rescue"
    EXPLORATION = "exploration"
    SURVEILLANCE = "surveillance"
    DELIVERY = "delivery"
    INSPECTION = "inspection"

class MissionState(Enum):
    PLANNING = "planning"
    READY = "ready"
    EXECUTING = "executing"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    ABORTED = "aborted"

class TaskType(Enum):
    SEARCH_AREA = "search_area"
    INSPECT_OBJECT = "inspect_object"
    DELIVER_ITEM = "deliver_item"
    MONITOR_LOCATION = "monitor_location"
    RETURN_BASE = "return_base"

@dataclass
class MissionTask:
    task_id: str
    task_type: TaskType
    target_location: Point
    priority: int
    estimated_duration: float
    dependencies: List[str]
    status: str = "pending"
    assigned_quadcopter: Optional[str] = None
    start_time: Optional[float] = None
    completion_time: Optional[float] = None

@dataclass
class Mission:
    mission_id: str
    mission_type: MissionType
    description: str
    tasks: List[MissionTask]
    start_location: Point
    target_area: List[Point]
    priority: int
    deadline: Optional[float] = None
    status: MissionState = MissionState.PLANNING
    created_time: float = 0.0
    start_time: Optional[float] = None
    completion_time: Optional[float] = None

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        
        # Mission management
        self.missions: Dict[str, Mission] = {}
        self.active_missions: Dict[str, Mission] = {}
        self.completed_missions: Dict[str, Mission] = {}
        self.failed_missions: Dict[str, Mission] = {}
        
        # Quadcopter management
        self.available_quadcopters: List[str] = []
        self.quadcopter_status: Dict[str, Dict] = {}
        self.quadcopter_locations: Dict[str, Point] = {}
        
        # Mission planning parameters
        self.planning_frequency = 5.0  # Hz
        self.task_timeout = 300.0  # seconds
        self.replanning_threshold = 0.3  # 30% deviation triggers replanning
        
        # Publishers
        self.mission_status_pub = self.create_publisher(String, '/quadcopter/mission/status', 10)
        self.mission_tasks_pub = self.create_publisher(String, '/quadcopter/mission/tasks', 10)
        self.mission_commands_pub = self.create_publisher(String, '/quadcopter/mission/commands', 10)
        self.mission_visualization_pub = self.create_publisher(MarkerArray, '/quadcopter/mission/visualization', 10)
        
        # Subscribers
        self.quadcopter_status_sub = self.create_subscription(
            String, '/quadcopter/status', self.quadcopter_status_callback, 10
        )
        self.quadcopter_location_sub = self.create_subscription(
            PoseStamped, '/quadcopter/pose', self.quadcopter_location_callback, 10
        )
        self.environment_update_sub = self.create_subscription(
            OccupancyGrid, '/quadcopter/map', self.environment_update_callback, 10
        )
        self.mission_request_sub = self.create_subscription(
            String, '/quadcopter/mission/request', self.mission_request_callback, 10
        )
        
        # Mission planning timer
        self.planning_timer = self.create_timer(1.0/self.planning_frequency, self.mission_planning_loop)
        
        # Initialize with sample missions
        self.initialize_sample_missions()
        
        self.get_logger().info("Mission Planner initialized")
        
    def initialize_sample_missions(self):
        """Initialize with sample search and rescue missions"""
        # Sample Search and Rescue Mission
        search_rescue_tasks = [
            MissionTask(
                task_id="search_zone_1",
                task_type=TaskType.SEARCH_AREA,
                target_location=Point(x=10.0, y=5.0, z=2.0),
                priority=1,
                estimated_duration=120.0,
                dependencies=[]
            ),
            MissionTask(
                task_id="search_zone_2",
                task_type=TaskType.SEARCH_AREA,
                target_location=Point(x=15.0, y=10.0, z=2.0),
                priority=2,
                estimated_duration=120.0,
                dependencies=["search_zone_1"]
            ),
            MissionTask(
                task_id="inspect_suspicious_object",
                task_type=TaskType.INSPECT_OBJECT,
                target_location=Point(x=12.0, y=7.0, z=1.5),
                priority=3,
                estimated_duration=60.0,
                dependencies=["search_zone_1", "search_zone_2"]
            ),
            MissionTask(
                task_id="return_to_base",
                task_type=TaskType.RETURN_BASE,
                target_location=Point(x=0.0, y=0.0, z=2.0),
                priority=4,
                estimated_duration=90.0,
                dependencies=["inspect_suspicious_object"]
            )
        ]
        
        search_rescue_mission = Mission(
            mission_id="search_rescue_001",
            mission_type=MissionType.SEARCH_AND_RESCUE,
            description="Search and rescue operation in building complex",
            tasks=search_rescue_tasks,
            start_location=Point(x=0.0, y=0.0, z=2.0),
            target_area=[
                Point(x=10.0, y=5.0, z=2.0),
                Point(x=15.0, y=10.0, z=2.0),
                Point(x=20.0, y=15.0, z=2.0)
            ],
            priority=1,
            deadline=time.time() + 3600,  # 1 hour deadline
            created_time=time.time()
        )
        
        self.missions[search_rescue_mission.mission_id] = search_rescue_mission
        
        # Sample Exploration Mission
        exploration_tasks = [
            MissionTask(
                task_id="explore_north",
                task_type=TaskType.SEARCH_AREA,
                target_location=Point(x=0.0, y=20.0, z=2.0),
                priority=1,
                estimated_duration=180.0,
                dependencies=[]
            ),
            MissionTask(
                task_id="explore_east",
                task_type=TaskType.SEARCH_AREA,
                target_location=Point(x=20.0, y=0.0, z=2.0),
                priority=2,
                estimated_duration=180.0,
                dependencies=[]
            ),
            MissionTask(
                task_id="explore_south",
                task_type=TaskType.SEARCH_AREA,
                target_location=Point(x=0.0, y=-20.0, z=2.0),
                priority=3,
                estimated_duration=180.0,
                dependencies=[]
            )
        ]
        
        exploration_mission = Mission(
            mission_id="exploration_001",
            mission_type=MissionType.EXPLORATION,
            description="Area exploration and mapping",
            tasks=exploration_tasks,
            start_location=Point(x=0.0, y=0.0, z=2.0),
            target_area=[
                Point(x=0.0, y=20.0, z=2.0),
                Point(x=20.0, y=0.0, z=2.0),
                Point(x=0.0, y=-20.0, z=2.0)
            ],
            priority=2,
            created_time=time.time()
        )
        
        self.missions[exploration_mission.mission_id] = exploration_mission
        
    def quadcopter_status_callback(self, msg):
        """Handle quadcopter status updates"""
        try:
            status_data = json.loads(msg.data)
            quadcopter_id = status_data.get('quadcopter_id', 'unknown')
            
            self.quadcopter_status[quadcopter_id] = status_data
            if quadcopter_id not in self.available_quadcopters:
                self.available_quadcopters.append(quadcopter_id)
                
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in quadcopter status")
            
    def quadcopter_location_callback(self, msg):
        """Handle quadcopter location updates"""
        # Extract quadcopter ID from frame_id or use default
        quadcopter_id = msg.header.frame_id if msg.header.frame_id != "map" else "quadcopter_1"
        
        self.quadcopter_locations[quadcopter_id] = msg.pose.position
        
    def environment_update_callback(self, msg):
        """Handle environment/map updates"""
        # Trigger replanning if significant changes detected
        self.check_environment_changes(msg)
        
    def mission_request_callback(self, msg):
        """Handle new mission requests"""
        try:
            request_data = json.loads(msg.data)
            mission_type = request_data.get('mission_type', 'exploration')
            description = request_data.get('description', 'New mission')
            target_area = request_data.get('target_area', [])
            
            # Create new mission
            new_mission = self.create_mission(mission_type, description, target_area)
            if new_mission:
                self.get_logger().info(f"New mission created: {new_mission.mission_id}")
                
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in mission request")
            
    def create_mission(self, mission_type: str, description: str, target_area: List[Dict]) -> Optional[Mission]:
        """Create a new mission based on request"""
        try:
            mission_type_enum = MissionType(mission_type)
        except ValueError:
            mission_type_enum = MissionType.EXPLORATION
            
        # Convert target area to Point objects
        target_points = []
        for area in target_area:
            point = Point(
                x=area.get('x', 0.0),
                y=area.get('y', 0.0),
                z=area.get('z', 2.0)
            )
            target_points.append(point)
            
        # Generate tasks based on mission type
        tasks = self.generate_tasks_for_mission(mission_type_enum, target_points)
        
        # Create mission
        mission = Mission(
            mission_id=f"{mission_type}_{int(time.time())}",
            mission_type=mission_type_enum,
            description=description,
            tasks=tasks,
            start_location=Point(x=0.0, y=0.0, z=2.0),
            target_area=target_points,
            priority=len(self.missions) + 1,
            created_time=time.time()
        )
        
        self.missions[mission.mission_id] = mission
        return mission
        
    def generate_tasks_for_mission(self, mission_type: MissionType, target_area: List[Point]) -> List[MissionTask]:
        """Generate appropriate tasks for mission type"""
        tasks = []
        
        if mission_type == MissionType.SEARCH_AND_RESCUE:
            # Create search tasks for each target area
            for i, target in enumerate(target_area):
                task = MissionTask(
                    task_id=f"search_zone_{i+1}",
                    task_type=TaskType.SEARCH_AREA,
                    target_location=target,
                    priority=i+1,
                    estimated_duration=120.0,
                    dependencies=[f"search_zone_{j+1}" for j in range(i)]
                )
                tasks.append(task)
                
            # Add return to base task
            return_task = MissionTask(
                task_id="return_to_base",
                task_type=TaskType.RETURN_BASE,
                target_location=Point(x=0.0, y=0.0, z=2.0),
                priority=len(tasks) + 1,
                estimated_duration=90.0,
                dependencies=[task.task_id for task in tasks]
            )
            tasks.append(return_task)
            
        elif mission_type == MissionType.EXPLORATION:
            # Create exploration tasks
            for i, target in enumerate(target_area):
                task = MissionTask(
                    task_id=f"explore_area_{i+1}",
                    task_type=TaskType.SEARCH_AREA,
                    target_location=target,
                    priority=i+1,
                    estimated_duration=180.0,
                    dependencies=[]
                )
                tasks.append(task)
                
        elif mission_type == MissionType.SURVEILLANCE:
            # Create monitoring tasks
            for i, target in enumerate(target_area):
                task = MissionTask(
                    task_id=f"monitor_location_{i+1}",
                    task_type=TaskType.MONITOR_LOCATION,
                    target_location=target,
                    priority=i+1,
                    estimated_duration=300.0,
                    dependencies=[]
                )
                tasks.append(task)
                
        return tasks
        
    def assign_tasks_to_quadcopters(self, mission: Mission):
        """Assign mission tasks to available quadcopters"""
        if not self.available_quadcopters:
            return
            
        # Simple round-robin assignment for now
        quadcopter_index = 0
        
        for task in mission.tasks:
            if task.status == "pending" and not task.assigned_quadcopter:
                quadcopter_id = self.available_quadcopters[quadcopter_index % len(self.available_quadcopters)]
                task.assigned_quadcopter = quadcopter_id
                quadcopter_index += 1
                
    def check_mission_progress(self, mission: Mission):
        """Check and update mission progress"""
        current_time = time.time()
        
        # Check task timeouts
        for task in mission.tasks:
            if task.status == "executing" and task.start_time:
                elapsed_time = current_time - task.start_time
                if elapsed_time > task.estimated_duration * 1.5:  # 50% over estimated time
                    self.get_logger().warn(f"Task {task.task_id} timeout, marking as failed")
                    task.status = "failed"
                    
        # Check mission completion
        completed_tasks = [task for task in mission.tasks if task.status == "completed"]
        failed_tasks = [task for task in mission.tasks if task.status == "failed"]
        
        if len(completed_tasks) == len(mission.tasks):
            mission.status = MissionState.COMPLETED
            mission.completion_time = current_time
            self.completed_missions[mission.mission_id] = mission
            if mission.mission_id in self.active_missions:
                del self.active_missions[mission.mission_id]
                
        elif len(failed_tasks) > len(mission.tasks) * 0.5:  # More than 50% failed
            mission.status = MissionState.FAILED
            self.failed_missions[mission.mission_id] = mission
            if mission.mission_id in self.active_missions:
                del self.active_missions[mission.mission_id]
                
    def check_environment_changes(self, map_msg: OccupancyGrid):
        """Check for significant environment changes that require replanning"""
        # Simple check: if map has changed significantly, trigger replanning
        # In a real implementation, this would compare with previous map state
        
        for mission_id, mission in self.active_missions.items():
            if mission.status == MissionState.EXECUTING:
                # Check if any tasks need replanning due to environment changes
                self.replan_mission_if_needed(mission, map_msg)
                
    def replan_mission_if_needed(self, mission: Mission, map_msg: OccupancyGrid):
        """Replan mission if environment changes require it"""
        # Check if target areas are still accessible
        for task in mission.tasks:
            if task.status == "pending":
                # Simple accessibility check (in real implementation, use path planning)
                if self.is_location_accessible(task.target_location, map_msg):
                    continue
                else:
                    self.get_logger().info(f"Location {task.target_location} no longer accessible, replanning")
                    self.replan_task(task, map_msg)
                    
    def is_location_accessible(self, location: Point, map_msg: OccupancyGrid) -> bool:
        """Check if a location is accessible given the current map"""
        # Simple implementation: check if location is within map bounds
        # In real implementation, use path planning to verify accessibility
        
        map_width = map_msg.info.width * map_msg.info.resolution
        map_height = map_msg.info.height * map_msg.info.resolution
        
        return (0 <= location.x <= map_width and 
                0 <= location.y <= map_height and
                0 <= location.z <= 10.0)  # Assume 10m height limit
                
    def replan_task(self, task: MissionTask, map_msg: OccupancyGrid):
        """Replan a specific task"""
        # Find alternative location or path
        # For now, just mark as failed and let mission planner handle it
        task.status = "failed"
        self.get_logger().info(f"Task {task.task_id} marked for replanning")
        
    def mission_planning_loop(self):
        """Main mission planning loop"""
        current_time = time.time()
        
        # Process pending missions
        for mission_id, mission in list(self.missions.items()):
            if mission.status == MissionState.PLANNING:
                # Assign tasks to quadcopters
                self.assign_tasks_to_quadcopters(mission)
                
                # Start mission if ready
                if all(task.assigned_quadcopter for task in mission.tasks):
                    mission.status = MissionState.READY
                    mission.start_time = current_time
                    self.active_missions[mission_id] = mission
                    self.get_logger().info(f"Mission {mission_id} started")
                    
        # Check active missions
        for mission_id, mission in list(self.active_missions.items()):
            if mission.status == MissionState.READY:
                mission.status = MissionState.EXECUTING
                
            if mission.status == MissionState.EXECUTING:
                self.check_mission_progress(mission)
                
        # Publish mission status
        self.publish_mission_status()
        
        # Publish mission visualization
        self.publish_mission_visualization()
        
    def publish_mission_status(self):
        """Publish current mission status"""
        status_data = {
            'timestamp': time.time(),
            'total_missions': len(self.missions),
            'active_missions': len(self.active_missions),
            'completed_missions': len(self.completed_missions),
            'failed_missions': len(self.failed_missions),
            'available_quadcopters': len(self.available_quadcopters),
            'missions': {}
        }
        
        # Add mission details
        for mission_id, mission in self.missions.items():
            status_data['missions'][mission_id] = {
                'type': mission.mission_type.value,
                'status': mission.status.value,
                'priority': mission.priority,
                'tasks': [
                    {
                        'task_id': task.task_id,
                        'type': task.task_type.value,
                        'status': task.status,
                        'assigned_quadcopter': task.assigned_quadcopter,
                        'priority': task.priority
                    }
                    for task in mission.tasks
                ]
            }
            
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.mission_status_pub.publish(status_msg)
        
    def publish_mission_visualization(self):
        """Publish mission visualization markers"""
        marker_array = MarkerArray()
        
        # Mission markers
        for mission_id, mission in self.active_missions.items():
            for i, task in enumerate(mission.tasks):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"mission_{mission_id}"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                marker.pose.position = task.target_location
                
                # Color based on task status
                if task.status == "completed":
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif task.status == "executing":
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                elif task.status == "failed":
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    
                marker.color.a = 0.8
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                
                marker_array.markers.append(marker)
                
        self.mission_visualization_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    mission_planner = MissionPlanner()
    
    try:
        rclpy.spin(mission_planner)
    except KeyboardInterrupt:
        mission_planner.get_logger().info("Mission Planner stopped by user")
    finally:
        mission_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 