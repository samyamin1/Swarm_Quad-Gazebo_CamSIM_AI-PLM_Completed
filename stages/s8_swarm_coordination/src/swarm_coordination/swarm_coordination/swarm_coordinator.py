#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import time
import random
import json
from threading import Lock
from enum import Enum
from collections import defaultdict

class SwarmState(Enum):
    IDLE = "idle"
    FORMATION = "formation"
    SEARCH = "search"
    COVERAGE = "coverage"
    RESCUE = "rescue"
    EMERGENCY = "emergency"

class FormationType(Enum):
    LINE = "line"
    CIRCLE = "circle"
    GRID = "grid"
    V_SHAPE = "v_shape"
    RANDOM = "random"

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')
        
        # Swarm parameters
        self.max_quadcopters = 5
        self.formation_spacing = 3.0  # meters
        self.safety_distance = 2.0  # meters
        self.coordination_frequency = 2.0  # Hz
        
        # Swarm state
        self.swarm_state = SwarmState.IDLE
        self.formation_type = FormationType.LINE
        self.quadcopters = {}  # {id: {'pose': {...}, 'status': {...}, 'goal': {...}}}
        self.swarm_center = {'x': 0.0, 'y': 0.0, 'z': 2.0}
        
        # AI coordination parameters
        self.coordination_confidence = 0.85
        self.processing_latency = 0.3  # seconds
        self.mission_progress = 0.0
        
        # Coordination state
        self.coordination_lock = Lock()
        self.mission_data = {}
        self.environment_data = {}
        self.last_coordination_time = 0
        self.coordination_count = 0
        
        # Publishers for each quadcopter
        self.quadcopter_publishers = {}
        for i in range(self.max_quadcopters):
            quad_id = f"quadcopter_{i+1}"
            self.quadcopter_publishers[quad_id] = {
                'mission_goal': self.create_publisher(PoseStamped, f'/{quad_id}/mission/goal', 10),
                'emergency': self.create_publisher(Bool, f'/{quad_id}/emergency', 10)
            }
            
        # Swarm-wide publishers
        self.swarm_status_pub = self.create_publisher(String, '/swarm/status', 10)
        self.coordination_log_pub = self.create_publisher(String, '/swarm/coordination/log', 10)
        self.formation_marker_pub = self.create_publisher(MarkerArray, '/swarm/formation/markers', 10)
        
        # Subscribers for each quadcopter
        self.quadcopter_subscribers = {}
        for i in range(self.max_quadcopters):
            quad_id = f"quadcopter_{i+1}"
            self.quadcopter_subscribers[quad_id] = {
                'status': self.create_subscription(
                    String, f'/{quad_id}/mission/status',
                    lambda msg, qid=quad_id: self.quadcopter_status_callback(qid, msg), 10
                ),
                'pose': self.create_subscription(
                    PoseStamped, f'/{quad_id}/slam/pose',
                    lambda msg, qid=quad_id: self.quadcopter_pose_callback(qid, msg), 10
                )
            }
            
        # Coordination timer
        self.coordination_timer = self.create_timer(1.0/self.coordination_frequency, self.update_coordination)
        
        # Formation planning
        self.formation_planner = FormationPlanner()
        self.mission_planner = SwarmMissionPlanner()
        self.collision_avoider = SwarmCollisionAvoider()
        
        self.get_logger().info("Swarm Coordinator initialized")
        
    def quadcopter_status_callback(self, quad_id, msg):
        """Process quadcopter status updates"""
        with self.coordination_lock:
            try:
                status = json.loads(msg.data)
                if quad_id not in self.quadcopters:
                    self.quadcopters[quad_id] = {}
                self.quadcopters[quad_id]['status'] = status
                self.quadcopters[quad_id]['last_update'] = time.time()
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON in {quad_id} status")
                
    def quadcopter_pose_callback(self, quad_id, msg):
        """Process quadcopter pose updates"""
        with self.coordination_lock:
            if quad_id not in self.quadcopters:
                self.quadcopters[quad_id] = {}
                
            self.quadcopters[quad_id]['pose'] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'timestamp': time.time()
            }
            
    def analyze_swarm_state(self):
        """Analyze current swarm state for coordination"""
        analysis = {
            'timestamp': time.time(),
            'swarm_state': self.swarm_state.value,
            'active_quadcopters': len(self.quadcopters),
            'formation_quality': 0.0,
            'collision_risk': 0.0,
            'mission_progress': self.mission_progress,
            'recommendations': []
        }
        
        # Analyze formation quality
        if len(self.quadcopters) > 1:
            formation_error = self.formation_planner.calculate_formation_error(
                self.quadcopters, self.formation_type
            )
            analysis['formation_quality'] = max(0.0, 1.0 - formation_error)
            
        # Analyze collision risk
        collision_risk = self.collision_avoider.calculate_collision_risk(self.quadcopters)
        analysis['collision_risk'] = collision_risk
        
        if collision_risk > 0.7:
            analysis['recommendations'].append("High collision risk - increase spacing")
            
        return analysis
        
    def make_coordination_decision(self, swarm_analysis):
        """Make AI-powered coordination decision"""
        # Simulate AI processing latency
        time.sleep(self.processing_latency)
        
        decision = {
            'timestamp': time.time(),
            'confidence': self.coordination_confidence,
            'action': None,
            'formation_type': self.formation_type.value,
            'quadcopter_goals': {},
            'reasoning': []
        }
        
        # Decision logic based on swarm state
        if self.swarm_state == SwarmState.IDLE:
            decision = self.handle_idle_state(swarm_analysis)
        elif self.swarm_state == SwarmState.FORMATION:
            decision = self.handle_formation_state(swarm_analysis)
        elif self.swarm_state == SwarmState.SEARCH:
            decision = self.handle_search_state(swarm_analysis)
        elif self.swarm_state == SwarmState.COVERAGE:
            decision = self.handle_coverage_state(swarm_analysis)
        elif self.swarm_state == SwarmState.RESCUE:
            decision = self.handle_rescue_state(swarm_analysis)
        elif self.swarm_state == SwarmState.EMERGENCY:
            decision = self.handle_emergency_state(swarm_analysis)
            
        return decision
        
    def handle_idle_state(self, analysis):
        """Handle idle state coordination"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.coordination_confidence,
            'action': 'start_formation',
            'formation_type': FormationType.LINE.value,
            'quadcopter_goals': {},
            'reasoning': ['Swarm ready', 'Starting formation phase']
        }
        
        # Start formation mission
        self.swarm_state = SwarmState.FORMATION
        self.formation_type = FormationType.LINE
        
        # Calculate formation goals
        formation_goals = self.formation_planner.calculate_formation_goals(
            self.quadcopters, self.formation_type, self.swarm_center
        )
        decision['quadcopter_goals'] = formation_goals
        
        return decision
        
    def handle_formation_state(self, analysis):
        """Handle formation state coordination"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.coordination_confidence,
            'action': 'maintain_formation',
            'formation_type': self.formation_type.value,
            'quadcopter_goals': {},
            'reasoning': []
        }
        
        # Check formation quality
        if analysis['formation_quality'] < 0.7:
            decision['action'] = 'adjust_formation'
            decision['reasoning'].append("Formation quality low - adjusting positions")
            
        # Check collision risk
        if analysis['collision_risk'] > 0.5:
            decision['action'] = 'avoid_collision'
            decision['reasoning'].append("High collision risk - avoiding")
            
        # Calculate formation goals
        formation_goals = self.formation_planner.calculate_formation_goals(
            self.quadcopters, self.formation_type, self.swarm_center
        )
        decision['quadcopter_goals'] = formation_goals
        
        # Check if formation is complete
        if analysis['formation_quality'] > 0.9:
            self.swarm_state = SwarmState.SEARCH
            decision['action'] = 'start_search'
            decision['reasoning'].append("Formation complete - starting search")
            
        return decision
        
    def handle_search_state(self, analysis):
        """Handle search state coordination"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.coordination_confidence,
            'action': 'coordinate_search',
            'formation_type': self.formation_type.value,
            'quadcopter_goals': {},
            'reasoning': []
        }
        
        # Coordinate search patterns
        search_goals = self.mission_planner.calculate_search_goals(
            self.quadcopters, self.swarm_center
        )
        decision['quadcopter_goals'] = search_goals
        
        # Check for targets found
        targets_found = self.check_for_targets()
        if targets_found:
            self.swarm_state = SwarmState.RESCUE
            decision['action'] = 'start_rescue'
            decision['reasoning'].append("Targets found - starting rescue")
            
        return decision
        
    def handle_coverage_state(self, analysis):
        """Handle coverage state coordination"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.coordination_confidence,
            'action': 'coordinate_coverage',
            'formation_type': self.formation_type.value,
            'quadcopter_goals': {},
            'reasoning': []
        }
        
        # Calculate coverage goals
        coverage_goals = self.mission_planner.calculate_coverage_goals(
            self.quadcopters, self.swarm_center
        )
        decision['quadcopter_goals'] = coverage_goals
        
        return decision
        
    def handle_rescue_state(self, analysis):
        """Handle rescue state coordination"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.coordination_confidence,
            'action': 'coordinate_rescue',
            'formation_type': self.formation_type.value,
            'quadcopter_goals': {},
            'reasoning': ['Targets acquired', 'Coordinating rescue']
        }
        
        # Calculate rescue goals
        rescue_goals = self.mission_planner.calculate_rescue_goals(
            self.quadcopters, self.swarm_center
        )
        decision['quadcopter_goals'] = rescue_goals
        
        # Simulate rescue completion
        if random.random() < 0.05:  # 5% chance of rescue completion
            self.swarm_state = SwarmState.IDLE
            decision['action'] = 'mission_complete'
            decision['reasoning'].append("Rescue completed - returning to idle")
            
        return decision
        
    def handle_emergency_state(self, analysis):
        """Handle emergency state coordination"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.coordination_confidence,
            'action': 'emergency_landing',
            'formation_type': self.formation_type.value,
            'quadcopter_goals': {},
            'reasoning': ['Emergency situation', 'Initiating emergency landing']
        }
        
        # Emergency landing goals
        for quad_id in self.quadcopters:
            decision['quadcopter_goals'][quad_id] = {
                'x': self.quadcopters[quad_id]['pose']['x'],
                'y': self.quadcopters[quad_id]['pose']['y'],
                'z': 0.5,  # Low hover
                'yaw': 0.0
            }
            
        return decision
        
    def check_for_targets(self):
        """Check if any quadcopter has found targets"""
        for quad_id, quad_data in self.quadcopters.items():
            if 'status' in quad_data:
                status = quad_data['status']
                if 'action' in status and 'approach_target' in status['action']:
                    return True
        return False
        
    def execute_coordination_decision(self, decision):
        """Execute the coordination decision"""
        # Publish swarm status
        status_msg = String()
        status_msg.data = json.dumps({
            'swarm_state': self.swarm_state.value,
            'action': decision['action'],
            'confidence': decision['confidence'],
            'formation_type': decision['formation_type'],
            'active_quadcopters': len(self.quadcopters),
            'reasoning': decision['reasoning']
        })
        self.swarm_status_pub.publish(status_msg)
        
        # Publish goals to each quadcopter
        for quad_id, goal in decision['quadcopter_goals'].items():
            if quad_id in self.quadcopter_publishers:
                goal_msg = PoseStamped()
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.header.frame_id = 'map'
                goal_msg.pose.position.x = goal['x']
                goal_msg.pose.position.y = goal['y']
                goal_msg.pose.position.z = goal['z']
                goal_msg.pose.orientation.w = 1.0
                
                self.quadcopter_publishers[quad_id]['mission_goal'].publish(goal_msg)
                
        # Publish formation markers
        self.publish_formation_markers()
        
        # Log coordination decision
        log_msg = String()
        log_msg.data = json.dumps(decision)
        self.coordination_log_pub.publish(log_msg)
        
    def publish_formation_markers(self):
        """Publish formation visualization markers"""
        marker_array = MarkerArray()
        
        # Formation center marker
        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "swarm_center"
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        
        center_marker.pose.position.x = self.swarm_center['x']
        center_marker.pose.position.y = self.swarm_center['y']
        center_marker.pose.position.z = self.swarm_center['z']
        
        center_marker.scale.x = 1.0
        center_marker.scale.y = 1.0
        center_marker.scale.z = 1.0
        center_marker.color.r = 0.0
        center_marker.color.g = 0.0
        center_marker.color.b = 1.0
        center_marker.color.a = 0.8
        
        marker_array.markers.append(center_marker)
        
        # Quadcopter position markers
        for i, (quad_id, quad_data) in enumerate(self.quadcopters.items()):
            if 'pose' in quad_data:
                pose = quad_data['pose']
                
                quad_marker = Marker()
                quad_marker.header.frame_id = "map"
                quad_marker.header.stamp = self.get_clock().now().to_msg()
                quad_marker.ns = "quadcopter_positions"
                quad_marker.id = i + 1
                quad_marker.type = Marker.CUBE
                quad_marker.action = Marker.ADD
                
                quad_marker.pose.position.x = pose['x']
                quad_marker.pose.position.y = pose['y']
                quad_marker.pose.position.z = pose['z']
                
                quad_marker.scale.x = 0.5
                quad_marker.scale.y = 0.5
                quad_marker.scale.z = 0.5
                quad_marker.color.r = 1.0
                quad_marker.color.g = 0.0
                quad_marker.color.b = 0.0
                quad_marker.color.a = 0.8
                
                marker_array.markers.append(quad_marker)
                
        self.formation_marker_pub.publish(marker_array)
        
    def update_coordination(self):
        """Main coordination update loop"""
        with self.coordination_lock:
            current_time = time.time()
            
            # Analyze swarm state
            swarm_analysis = self.analyze_swarm_state()
            
            # Make coordination decision
            decision = self.make_coordination_decision(swarm_analysis)
            
            # Execute decision
            self.execute_coordination_decision(decision)
            
            self.last_coordination_time = current_time
            self.coordination_count += 1
            
            # Log coordination
            self.get_logger().info(f"Coordination #{self.coordination_count}: {decision['action']} "
                                 f"(confidence: {decision['confidence']:.2f}, "
                                 f"quadcopters: {len(self.quadcopters)})")

class FormationPlanner:
    def __init__(self):
        self.spacing = 3.0
        
    def calculate_formation_goals(self, quadcopters, formation_type, center):
        """Calculate formation goals for all quadcopters"""
        goals = {}
        
        if formation_type == FormationType.LINE:
            goals = self.calculate_line_formation(quadcopters, center)
        elif formation_type == FormationType.CIRCLE:
            goals = self.calculate_circle_formation(quadcopters, center)
        elif formation_type == FormationType.GRID:
            goals = self.calculate_grid_formation(quadcopters, center)
        elif formation_type == FormationType.V_SHAPE:
            goals = self.calculate_v_formation(quadcopters, center)
        else:
            goals = self.calculate_random_formation(quadcopters, center)
            
        return goals
        
    def calculate_line_formation(self, quadcopters, center):
        """Calculate line formation goals"""
        goals = {}
        quad_ids = list(quadcopters.keys())
        
        for i, quad_id in enumerate(quad_ids):
            offset = (i - len(quad_ids) // 2) * self.spacing
            goals[quad_id] = {
                'x': center['x'] + offset,
                'y': center['y'],
                'z': center['z'],
                'yaw': 0.0
            }
            
        return goals
        
    def calculate_circle_formation(self, quadcopters, center):
        """Calculate circle formation goals"""
        goals = {}
        quad_ids = list(quadcopters.keys())
        
        for i, quad_id in enumerate(quad_ids):
            angle = 2 * np.pi * i / len(quad_ids)
            goals[quad_id] = {
                'x': center['x'] + self.spacing * np.cos(angle),
                'y': center['y'] + self.spacing * np.sin(angle),
                'z': center['z'],
                'yaw': angle
            }
            
        return goals
        
    def calculate_grid_formation(self, quadcopters, center):
        """Calculate grid formation goals"""
        goals = {}
        quad_ids = list(quadcopters.keys())
        
        grid_size = int(np.ceil(np.sqrt(len(quad_ids))))
        for i, quad_id in enumerate(quad_ids):
            row = i // grid_size
            col = i % grid_size
            goals[quad_id] = {
                'x': center['x'] + (col - grid_size // 2) * self.spacing,
                'y': center['y'] + (row - grid_size // 2) * self.spacing,
                'z': center['z'],
                'yaw': 0.0
            }
            
        return goals
        
    def calculate_v_formation(self, quadcopters, center):
        """Calculate V formation goals"""
        goals = {}
        quad_ids = list(quadcopters.keys())
        
        for i, quad_id in enumerate(quad_ids):
            if i == 0:  # Leader
                goals[quad_id] = {
                    'x': center['x'],
                    'y': center['y'],
                    'z': center['z'],
                    'yaw': 0.0
                }
            else:  # Followers
                side = 1 if i % 2 == 1 else -1
                row = (i - 1) // 2 + 1
                goals[quad_id] = {
                    'x': center['x'] - row * self.spacing * 0.5,
                    'y': center['y'] + side * row * self.spacing * 0.8,
                    'z': center['z'],
                    'yaw': 0.0
                }
                
        return goals
        
    def calculate_random_formation(self, quadcopters, center):
        """Calculate random formation goals"""
        goals = {}
        
        for quad_id in quadcopters:
            goals[quad_id] = {
                'x': center['x'] + random.uniform(-self.spacing, self.spacing),
                'y': center['y'] + random.uniform(-self.spacing, self.spacing),
                'z': center['z'],
                'yaw': random.uniform(0, 2 * np.pi)
            }
            
        return goals
        
    def calculate_formation_error(self, quadcopters, formation_type):
        """Calculate formation error"""
        if len(quadcopters) < 2:
            return 0.0
            
        goals = self.calculate_formation_goals(quadcopters, formation_type, {'x': 0, 'y': 0, 'z': 2})
        total_error = 0.0
        
        for quad_id, goal in goals.items():
            if quad_id in quadcopters and 'pose' in quadcopters[quad_id]:
                pose = quadcopters[quad_id]['pose']
                error = np.sqrt(
                    (pose['x'] - goal['x'])**2 +
                    (pose['y'] - goal['y'])**2 +
                    (pose['z'] - goal['z'])**2
                )
                total_error += error
                
        return total_error / len(quadcopters)

class SwarmMissionPlanner:
    def __init__(self):
        self.search_patterns = ['spiral', 'grid', 'random']
        
    def calculate_search_goals(self, quadcopters, center):
        """Calculate search mission goals"""
        goals = {}
        quad_ids = list(quadcopters.keys())
        
        for i, quad_id in enumerate(quad_ids):
            # Spiral search pattern
            angle = 2 * np.pi * i / len(quad_ids)
            radius = 5.0 + i * 2.0
            goals[quad_id] = {
                'x': center['x'] + radius * np.cos(angle),
                'y': center['y'] + radius * np.sin(angle),
                'z': center['z'],
                'yaw': angle
            }
            
        return goals
        
    def calculate_coverage_goals(self, quadcopters, center):
        """Calculate coverage mission goals"""
        goals = {}
        quad_ids = list(quadcopters.keys())
        
        for i, quad_id in enumerate(quad_ids):
            # Grid coverage pattern
            grid_size = int(np.ceil(np.sqrt(len(quad_ids))))
            row = i // grid_size
            col = i % grid_size
            goals[quad_id] = {
                'x': center['x'] + (col - grid_size // 2) * 4.0,
                'y': center['y'] + (row - grid_size // 2) * 4.0,
                'z': center['z'],
                'yaw': 0.0
            }
            
        return goals
        
    def calculate_rescue_goals(self, quadcopters, center):
        """Calculate rescue mission goals"""
        goals = {}
        quad_ids = list(quadcopters.keys())
        
        for i, quad_id in enumerate(quadcopters):
            # Concentrate around target
            angle = 2 * np.pi * i / len(quad_ids)
            radius = 2.0
            goals[quad_id] = {
                'x': center['x'] + radius * np.cos(angle),
                'y': center['y'] + radius * np.sin(angle),
                'z': center['z'] - 1.0,  # Lower altitude for rescue
                'yaw': angle
            }
            
        return goals

class SwarmCollisionAvoider:
    def __init__(self):
        self.safety_distance = 2.0
        
    def calculate_collision_risk(self, quadcopters):
        """Calculate collision risk in swarm"""
        if len(quadcopters) < 2:
            return 0.0
            
        total_risk = 0.0
        pairs = 0
        
        quad_ids = list(quadcopters.keys())
        for i in range(len(quad_ids)):
            for j in range(i + 1, len(quad_ids)):
                quad1_id = quad_ids[i]
                quad2_id = quad_ids[j]
                
                if (quad1_id in quadcopters and quad2_id in quadcopters and
                    'pose' in quadcopters[quad1_id] and 'pose' in quadcopters[quad2_id]):
                    
                    pose1 = quadcopters[quad1_id]['pose']
                    pose2 = quadcopters[quad2_id]['pose']
                    
                    distance = np.sqrt(
                        (pose1['x'] - pose2['x'])**2 +
                        (pose1['y'] - pose2['y'])**2 +
                        (pose1['z'] - pose2['z'])**2
                    )
                    
                    if distance < self.safety_distance:
                        risk = 1.0 - (distance / self.safety_distance)
                        total_risk += risk
                    pairs += 1
                    
        return total_risk / max(pairs, 1)

def main(args=None):
    rclpy.init(args=args)
    swarm_coordinator = SwarmCoordinator()
    
    try:
        rclpy.spin(swarm_coordinator)
    except KeyboardInterrupt:
        swarm_coordinator.get_logger().info("Swarm Coordinator stopped by user")
    finally:
        swarm_coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 