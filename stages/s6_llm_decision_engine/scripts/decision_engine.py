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

class MissionState(Enum):
    IDLE = "idle"
    EXPLORATION = "exploration"
    SEARCH = "search"
    RESCUE = "rescue"
    RETURN = "return"
    EMERGENCY = "emergency"

class DecisionEngine(Node):
    def __init__(self):
        super().__init__('llm_decision_engine')
        
        # AI Decision parameters
        self.decision_frequency = 1.0  # Hz
        self.confidence_threshold = 0.7
        self.emergency_threshold = 0.9
        
        # Mission parameters
        self.mission_state = MissionState.IDLE
        self.mission_goals = []
        self.current_goal = None
        self.mission_progress = 0.0
        
        # AI model state (simulated)
        self.model_loaded = True
        self.processing_latency = 0.2  # seconds
        self.model_confidence = 0.85
        
        # Decision state
        self.decision_lock = Lock()
        self.environment_data = {}
        self.sensor_data = {}
        self.slam_data = {}
        self.perception_data = {}
        self.last_decision_time = 0
        self.decision_count = 0
        
        # Publishers
        self.mission_goal_pub = self.create_publisher(PoseStamped, '/quadcopter/mission/goal', 10)
        self.navigation_cmd_pub = self.create_publisher(Twist, '/quadcopter/navigation/command', 10)
        self.mission_status_pub = self.create_publisher(String, '/quadcopter/mission/status', 10)
        self.decision_log_pub = self.create_publisher(String, '/quadcopter/decision/log', 10)
        self.emergency_pub = self.create_publisher(Bool, '/quadcopter/emergency', 10)
        
        # Subscribers
        self.perception_sub = self.create_subscription(
            String, '/quadcopter/perception/description', 
            self.perception_callback, 10
        )
        self.analysis_sub = self.create_subscription(
            String, '/quadcopter/perception/analysis', 
            self.analysis_callback, 10
        )
        self.alerts_sub = self.create_subscription(
            String, '/quadcopter/perception/alerts', 
            self.alerts_callback, 10
        )
        self.slam_pose_sub = self.create_subscription(
            PoseStamped, '/quadcopter/slam/pose', 
            self.slam_pose_callback, 10
        )
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid, '/quadcopter/slam/map', 
            self.slam_map_callback, 10
        )
        
        # Decision timer
        self.decision_timer = self.create_timer(1.0/self.decision_frequency, self.update_decisions)
        
        # Mission planning
        self.mission_planner = MissionPlanner()
        self.path_planner = PathPlanner()
        self.obstacle_avoider = ObstacleAvoider()
        
        self.get_logger().info("LLM Decision Engine initialized")
        
    def perception_callback(self, msg):
        """Process perception descriptions"""
        with self.decision_lock:
            self.perception_data['description'] = msg.data
            self.perception_data['timestamp'] = time.time()
            
    def analysis_callback(self, msg):
        """Process AI analysis"""
        with self.decision_lock:
            try:
                analysis = json.loads(msg.data)
                self.perception_data['analysis'] = analysis
                self.perception_data['analysis_timestamp'] = time.time()
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in analysis message")
                
    def alerts_callback(self, msg):
        """Process critical alerts"""
        with self.decision_lock:
            try:
                alerts = json.loads(msg.data)
                self.perception_data['alerts'] = alerts
                self.perception_data['alerts_timestamp'] = time.time()
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in alerts message")
                
    def slam_pose_callback(self, msg):
        """Process SLAM pose data"""
        with self.decision_lock:
            self.slam_data['pose'] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'timestamp': time.time()
            }
            
    def slam_map_callback(self, msg):
        """Process SLAM map data"""
        with self.decision_lock:
            self.slam_data['map'] = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'timestamp': time.time()
            }
            
    def analyze_environment(self):
        """Analyze current environment for decision making"""
        analysis = {
            'timestamp': time.time(),
            'mission_state': self.mission_state.value,
            'environment_quality': 0.0,
            'safety_score': 0.0,
            'exploration_progress': 0.0,
            'recommendations': []
        }
        
        # Analyze perception data
        if 'analysis' in self.perception_data:
            ai_analysis = self.perception_data['analysis']
            analysis['environment_quality'] = ai_analysis.get('confidence', 0.0)
            
            # Check for critical alerts
            if 'alerts' in self.perception_data:
                alerts = self.perception_data['alerts']
                critical_alerts = [alert for alert in alerts if alert.get('severity') == 'high']
                if critical_alerts:
                    analysis['safety_score'] = 0.3  # Low safety
                    analysis['recommendations'].append("Emergency situation detected - prioritize safety")
                else:
                    analysis['safety_score'] = 0.8  # Good safety
                    
        # Analyze SLAM data
        if 'pose' in self.slam_data and 'map' in self.slam_data:
            # Calculate exploration progress
            map_area = self.slam_data['map']['width'] * self.slam_data['map']['height']
            analysis['exploration_progress'] = min(1.0, self.decision_count / 100.0)  # Simulated progress
            
        return analysis
        
    def make_decision(self, environment_analysis):
        """Make AI-powered decision based on environment analysis"""
        # Simulate AI processing latency
        time.sleep(self.processing_latency)
        
        decision = {
            'timestamp': time.time(),
            'confidence': self.model_confidence,
            'action': None,
            'reasoning': [],
            'target_pose': None,
            'emergency': False
        }
        
        # Decision logic based on mission state and environment
        if self.mission_state == MissionState.IDLE:
            decision = self.handle_idle_state(environment_analysis)
        elif self.mission_state == MissionState.EXPLORATION:
            decision = self.handle_exploration_state(environment_analysis)
        elif self.mission_state == MissionState.SEARCH:
            decision = self.handle_search_state(environment_analysis)
        elif self.mission_state == MissionState.RESCUE:
            decision = self.handle_rescue_state(environment_analysis)
        elif self.mission_state == MissionState.RETURN:
            decision = self.handle_return_state(environment_analysis)
        elif self.mission_state == MissionState.EMERGENCY:
            decision = self.handle_emergency_state(environment_analysis)
            
        return decision
        
    def handle_idle_state(self, analysis):
        """Handle idle state decisions"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.model_confidence,
            'action': 'start_exploration',
            'reasoning': ['System ready', 'No active mission', 'Starting exploration phase'],
            'target_pose': None,
            'emergency': False
        }
        
        # Start exploration mission
        self.mission_state = MissionState.EXPLORATION
        self.mission_goals = self.mission_planner.create_exploration_goals()
        
        return decision
        
    def handle_exploration_state(self, analysis):
        """Handle exploration state decisions"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.model_confidence,
            'action': 'explore_area',
            'reasoning': [],
            'target_pose': None,
            'emergency': False
        }
        
        # Check for safety issues
        if analysis['safety_score'] < 0.5:
            decision['action'] = 'avoid_obstacle'
            decision['reasoning'].append("Low safety score - avoiding obstacles")
            decision['emergency'] = True
        else:
            # Continue exploration
            if self.current_goal is None and self.mission_goals:
                self.current_goal = self.mission_goals.pop(0)
                
            if self.current_goal:
                decision['target_pose'] = self.current_goal
                decision['reasoning'].append(f"Moving to exploration goal: {self.current_goal}")
            else:
                # Exploration complete
                self.mission_state = MissionState.SEARCH
                decision['action'] = 'start_search'
                decision['reasoning'].append("Exploration complete - starting search phase")
                
        return decision
        
    def handle_search_state(self, analysis):
        """Handle search state decisions"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.model_confidence,
            'action': 'search_area',
            'reasoning': [],
            'target_pose': None,
            'emergency': False
        }
        
        # Check for targets in perception data
        if 'description' in self.perception_data:
            description = self.perception_data['description'].lower()
            if 'person' in description or 'human' in description:
                decision['action'] = 'approach_target'
                decision['reasoning'].append("Human detected - approaching for rescue")
                self.mission_state = MissionState.RESCUE
            else:
                decision['reasoning'].append("No targets found - continuing search")
                
        return decision
        
    def handle_rescue_state(self, analysis):
        """Handle rescue state decisions"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.model_confidence,
            'action': 'execute_rescue',
            'reasoning': ['Target acquired', 'Executing rescue protocol'],
            'target_pose': None,
            'emergency': False
        }
        
        # Simulate rescue completion
        if random.random() < 0.1:  # 10% chance of rescue completion
            self.mission_state = MissionState.RETURN
            decision['action'] = 'return_to_base'
            decision['reasoning'].append("Rescue completed - returning to base")
            
        return decision
        
    def handle_return_state(self, analysis):
        """Handle return state decisions"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.model_confidence,
            'action': 'return_to_base',
            'reasoning': ['Mission completed', 'Returning to base'],
            'target_pose': {'x': 0, 'y': 0, 'z': 2},  # Return to origin
            'emergency': False
        }
        
        # Check if returned to base
        if 'pose' in self.slam_data:
            pose = self.slam_data['pose']
            distance_to_base = np.sqrt(pose['x']**2 + pose['y']**2 + (pose['z'] - 2)**2)
            if distance_to_base < 1.0:
                self.mission_state = MissionState.IDLE
                decision['action'] = 'mission_complete'
                decision['reasoning'].append("Successfully returned to base")
                
        return decision
        
    def handle_emergency_state(self, analysis):
        """Handle emergency state decisions"""
        decision = {
            'timestamp': time.time(),
            'confidence': self.model_confidence,
            'action': 'emergency_landing',
            'reasoning': ['Emergency situation detected', 'Initiating emergency landing'],
            'target_pose': None,
            'emergency': True
        }
        
        # Check if emergency resolved
        if analysis['safety_score'] > 0.7:
            self.mission_state = MissionState.EXPLORATION
            decision['action'] = 'resume_mission'
            decision['reasoning'].append("Emergency resolved - resuming mission")
            decision['emergency'] = False
            
        return decision
        
    def execute_decision(self, decision):
        """Execute the AI decision"""
        # Publish mission status
        status_msg = String()
        status_msg.data = json.dumps({
            'mission_state': self.mission_state.value,
            'action': decision['action'],
            'confidence': decision['confidence'],
            'reasoning': decision['reasoning']
        })
        self.mission_status_pub.publish(status_msg)
        
        # Publish navigation command
        if decision['target_pose']:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = decision['target_pose']['x']
            goal_msg.pose.position.y = decision['target_pose']['y']
            goal_msg.pose.position.z = decision['target_pose']['z']
            goal_msg.pose.orientation.w = 1.0
            self.mission_goal_pub.publish(goal_msg)
            
        # Publish emergency status
        emergency_msg = Bool()
        emergency_msg.data = decision['emergency']
        self.emergency_pub.publish(emergency_msg)
        
        # Log decision
        log_msg = String()
        log_msg.data = json.dumps(decision)
        self.decision_log_pub.publish(log_msg)
        
    def update_decisions(self):
        """Main decision update loop"""
        with self.decision_lock:
            current_time = time.time()
            
            # Analyze environment
            environment_analysis = self.analyze_environment()
            
            # Make AI decision
            decision = self.make_decision(environment_analysis)
            
            # Execute decision
            self.execute_decision(decision)
            
            self.last_decision_time = current_time
            self.decision_count += 1
            
            # Log decision
            self.get_logger().info(f"Decision #{self.decision_count}: {decision['action']} "
                                 f"(confidence: {decision['confidence']:.2f})")

class MissionPlanner:
    def __init__(self):
        self.exploration_goals = []
        
    def create_exploration_goals(self):
        """Create exploration mission goals"""
        goals = []
        
        # Create a grid of exploration points
        for x in range(-10, 11, 5):
            for y in range(-10, 11, 5):
                goal = {
                    'x': x,
                    'y': y,
                    'z': 2.0
                }
                goals.append(goal)
                
        # Shuffle goals for random exploration
        random.shuffle(goals)
        return goals

class PathPlanner:
    def __init__(self):
        self.current_path = []
        
    def plan_path(self, start, goal):
        """Plan path from start to goal"""
        # Simple straight-line path planning
        path = [start, goal]
        return path

class ObstacleAvoider:
    def __init__(self):
        self.safety_margin = 1.0
        
    def check_collision(self, pose, obstacles):
        """Check for potential collisions"""
        # Simple collision detection
        for obstacle in obstacles:
            distance = np.sqrt((pose['x'] - obstacle['x'])**2 + 
                             (pose['y'] - obstacle['y'])**2)
            if distance < self.safety_margin:
                return True
        return False

def main(args=None):
    rclpy.init(args=args)
    decision_engine = DecisionEngine()
    
    try:
        rclpy.spin(decision_engine)
    except KeyboardInterrupt:
        decision_engine.get_logger().info("LLM Decision Engine stopped by user")
    finally:
        decision_engine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 