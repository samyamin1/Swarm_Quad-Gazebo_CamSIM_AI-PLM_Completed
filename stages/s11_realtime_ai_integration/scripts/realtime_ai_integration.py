#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray, Marker
import time
import json
import numpy as np
from threading import Lock
from enum import Enum
from collections import defaultdict
import asyncio
import concurrent.futures

class AIComponentState(Enum):
    INITIALIZING = "initializing"
    READY = "ready"
    PROCESSING = "processing"
    ERROR = "error"
    OFFLINE = "offline"

class AIComponentType(Enum):
    PERCEPTION = "perception"
    DECISION = "decision"
    SLAM = "slam"
    FUSION = "fusion"
    NAVIGATION = "navigation"
    SWARM = "swarm"

class RealTimeAIIntegration(Node):
    def __init__(self):
        super().__init__('realtime_ai_integration')
        
        # AI Integration parameters
        self.integration_frequency = 20.0  # Hz
        self.component_timeout = 2.0  # seconds
        self.max_processing_time = 0.1  # seconds
        self.confidence_threshold = 0.7
        
        # AI component tracking
        self.ai_components = {}
        self.component_states = {}
        self.component_timestamps = {}
        self.component_data = {}
        self.component_health = {}
        
        # Integration state
        self.integration_state = AIComponentState.INITIALIZING
        self.last_integration_time = 0
        self.processing_queue = []
        self.results_cache = {}
        
        # Thread safety
        self.integration_lock = Lock()
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)
        
        # Publishers
        self.ai_status_pub = self.create_publisher(String, '/quadcopter/ai/status', 10)
        self.ai_results_pub = self.create_publisher(String, '/quadcopter/ai/results', 10)
        self.ai_commands_pub = self.create_publisher(String, '/quadcopter/ai/commands', 10)
        self.ai_visualization_pub = self.create_publisher(MarkerArray, '/quadcopter/ai/visualization', 10)
        
        # Subscribers for AI component data
        self.setup_ai_subscribers()
        
        # Integration timer
        self.integration_timer = self.create_timer(1.0/self.integration_frequency, self.integration_loop)
        
        # AI coordinators
        self.perception_coordinator = PerceptionCoordinator()
        self.decision_coordinator = DecisionCoordinator()
        self.slam_coordinator = SLAMCoordinator()
        self.fusion_coordinator = FusionCoordinator()
        self.navigation_coordinator = NavigationCoordinator()
        self.swarm_coordinator = SwarmCoordinator()
        
        self.get_logger().info("Real-time AI Integration initialized")
        
    def setup_ai_subscribers(self):
        """Setup subscribers for AI component data"""
        # Perception data
        self.perception_description_sub = self.create_subscription(
            String, '/quadcopter/perception/description',
            lambda msg: self.ai_component_callback(AIComponentType.PERCEPTION, 'description', msg), 10
        )
        self.perception_analysis_sub = self.create_subscription(
            String, '/quadcopter/perception/analysis',
            lambda msg: self.ai_component_callback(AIComponentType.PERCEPTION, 'analysis', msg), 10
        )
        self.perception_alerts_sub = self.create_subscription(
            String, '/quadcopter/perception/alerts',
            lambda msg: self.ai_component_callback(AIComponentType.PERCEPTION, 'alerts', msg), 10
        )
        
        # Decision data
        self.decision_status_sub = self.create_subscription(
            String, '/quadcopter/mission/status',
            lambda msg: self.ai_component_callback(AIComponentType.DECISION, 'status', msg), 10
        )
        self.decision_log_sub = self.create_subscription(
            String, '/quadcopter/decision/log',
            lambda msg: self.ai_component_callback(AIComponentType.DECISION, 'log', msg), 10
        )
        
        # SLAM data
        self.slam_pose_sub = self.create_subscription(
            PoseStamped, '/quadcopter/slam/pose',
            lambda msg: self.ai_component_callback(AIComponentType.SLAM, 'pose', msg), 10
        )
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid, '/quadcopter/slam/map',
            lambda msg: self.ai_component_callback(AIComponentType.SLAM, 'map', msg), 10
        )
        self.slam_status_sub = self.create_subscription(
            String, '/quadcopter/slam/status',
            lambda msg: self.ai_component_callback(AIComponentType.SLAM, 'status', msg), 10
        )
        
        # Fusion data
        self.fusion_data_sub = self.create_subscription(
            String, '/quadcopter/fusion/data',
            lambda msg: self.ai_component_callback(AIComponentType.FUSION, 'data', msg), 10
        )
        self.fusion_status_sub = self.create_subscription(
            String, '/quadcopter/fusion/status',
            lambda msg: self.ai_component_callback(AIComponentType.FUSION, 'status', msg), 10
        )
        
        # Navigation data
        self.navigation_status_sub = self.create_subscription(
            String, '/quadcopter/navigation/status',
            lambda msg: self.ai_component_callback(AIComponentType.NAVIGATION, 'status', msg), 10
        )
        
        # Swarm data
        self.swarm_status_sub = self.create_subscription(
            String, '/quadcopter/swarm/status',
            lambda msg: self.ai_component_callback(AIComponentType.SWARM, 'status', msg), 10
        )
        
    def ai_component_callback(self, component_type, data_type, msg):
        """Process AI component data"""
        with self.integration_lock:
            current_time = time.time()
            component_key = f"{component_type.value}_{data_type}"
            
            # Update component health
            self.component_health[component_key] = True
            self.component_timestamps[component_key] = current_time
            
            # Store component data
            if component_type not in self.component_data:
                self.component_data[component_type] = {}
                
            self.component_data[component_type][data_type] = {
                'data': msg,
                'timestamp': current_time,
                'processed': False
            }
            
            # Update component state
            if component_type not in self.component_states:
                self.component_states[component_type] = AIComponentState.READY
                
    def check_component_health(self):
        """Check health of all AI components"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.component_health)
        
        for component_key, is_healthy in self.component_health.items():
            if component_key in self.component_timestamps:
                time_since_update = current_time - self.component_timestamps[component_key]
                if time_since_update < self.component_timeout:
                    healthy_components += 1
                else:
                    self.component_health[component_key] = False
                    
        return healthy_components, total_components
        
    def coordinate_ai_components(self):
        """Coordinate all AI components for real-time operation"""
        coordination_result = {
            'timestamp': time.time(),
            'integration_state': self.integration_state.value,
            'component_count': len(self.component_data),
            'healthy_components': 0,
            'coordination_results': {},
            'ai_commands': [],
            'confidence': 0.0
        }
        
        # Check component health
        healthy_components, total_components = self.check_component_health()
        coordination_result['healthy_components'] = healthy_components
        coordination_result['confidence'] = healthy_components / max(total_components, 1)
        
        # Coordinate each AI component type
        for component_type in AIComponentType:
            if component_type in self.component_data:
                coordinator_result = self.coordinate_component(component_type)
                coordination_result['coordination_results'][component_type.value] = coordinator_result
                
        # Generate AI commands based on coordination
        ai_commands = self.generate_ai_commands(coordination_result)
        coordination_result['ai_commands'] = ai_commands
        
        return coordination_result
        
    def coordinate_component(self, component_type):
        """Coordinate a specific AI component"""
        if component_type == AIComponentType.PERCEPTION:
            return self.perception_coordinator.coordinate(self.component_data[component_type])
        elif component_type == AIComponentType.DECISION:
            return self.decision_coordinator.coordinate(self.component_data[component_type])
        elif component_type == AIComponentType.SLAM:
            return self.slam_coordinator.coordinate(self.component_data[component_type])
        elif component_type == AIComponentType.FUSION:
            return self.fusion_coordinator.coordinate(self.component_data[component_type])
        elif component_type == AIComponentType.NAVIGATION:
            return self.navigation_coordinator.coordinate(self.component_data[component_type])
        elif component_type == AIComponentType.SWARM:
            return self.swarm_coordinator.coordinate(self.component_data[component_type])
        else:
            return {'status': 'unknown', 'confidence': 0.0}
            
    def generate_ai_commands(self, coordination_result):
        """Generate AI commands based on coordination results"""
        commands = []
        
        # Check for critical alerts
        if 'perception' in coordination_result['coordination_results']:
            perception_result = coordination_result['coordination_results']['perception']
            if 'alerts' in perception_result and perception_result['alerts']:
                commands.append({
                    'type': 'emergency_response',
                    'priority': 'high',
                    'action': 'immediate_landing',
                    'reason': 'Critical alert detected'
                })
                
        # Check for navigation issues
        if 'navigation' in coordination_result['coordination_results']:
            navigation_result = coordination_result['coordination_results']['navigation']
            if navigation_result.get('confidence', 0) < 0.5:
                commands.append({
                    'type': 'navigation_adjustment',
                    'priority': 'medium',
                    'action': 'recalculate_path',
                    'reason': 'Low navigation confidence'
                })
                
        # Check for SLAM issues
        if 'slam' in coordination_result['coordination_results']:
            slam_result = coordination_result['coordination_results']['slam']
            if slam_result.get('confidence', 0) < 0.6:
                commands.append({
                    'type': 'slam_recovery',
                    'priority': 'medium',
                    'action': 'relocalize',
                    'reason': 'SLAM confidence low'
                })
                
        # Check for decision making
        if 'decision' in coordination_result['coordination_results']:
            decision_result = coordination_result['coordination_results']['decision']
            if decision_result.get('confidence', 0) > 0.8:
                commands.append({
                    'type': 'mission_execution',
                    'priority': 'normal',
                    'action': 'continue_mission',
                    'reason': 'High decision confidence'
                })
                
        return commands
        
    def publish_integration_results(self, coordination_result):
        """Publish AI integration results"""
        # Publish AI status
        status_msg = String()
        status_data = {
            'integration_state': coordination_result['integration_state'],
            'healthy_components': coordination_result['healthy_components'],
            'total_components': coordination_result['component_count'],
            'confidence': coordination_result['confidence'],
            'timestamp': coordination_result['timestamp']
        }
        status_msg.data = json.dumps(status_data)
        self.ai_status_pub.publish(status_msg)
        
        # Publish AI results
        results_msg = String()
        results_msg.data = json.dumps(coordination_result)
        self.ai_results_pub.publish(results_msg)
        
        # Publish AI commands
        if coordination_result['ai_commands']:
            commands_msg = String()
            commands_msg.data = json.dumps(coordination_result['ai_commands'])
            self.ai_commands_pub.publish(commands_msg)
            
        # Publish visualization
        self.publish_ai_visualization(coordination_result)
        
    def publish_ai_visualization(self, coordination_result):
        """Publish AI visualization markers"""
        marker_array = MarkerArray()
        
        # Component health markers
        for i, (component_key, is_healthy) in enumerate(self.component_health.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ai_components"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = i * 3.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            if is_healthy:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
        # Confidence indicator
        confidence_marker = Marker()
        confidence_marker.header.frame_id = "map"
        confidence_marker.header.stamp = self.get_clock().now().to_msg()
        confidence_marker.ns = "ai_confidence"
        confidence_marker.id = 100
        confidence_marker.type = Marker.CYLINDER
        confidence_marker.action = Marker.ADD
        
        confidence_marker.pose.position.x = 0.0
        confidence_marker.pose.position.y = 5.0
        confidence_marker.pose.position.z = coordination_result['confidence'] * 2.0
        
        confidence_marker.scale.x = 1.0
        confidence_marker.scale.y = 1.0
        confidence_marker.scale.z = 2.0
        
        confidence_marker.color.r = 0.0
        confidence_marker.color.g = 0.0
        confidence_marker.color.b = 1.0
        confidence_marker.color.a = 0.6
        marker_array.markers.append(confidence_marker)
        
        self.ai_visualization_pub.publish(marker_array)
        
    def integration_loop(self):
        """Main AI integration loop"""
        with self.integration_lock:
            current_time = time.time()
            
            # Check if we have enough AI components
            if len(self.component_data) < 3:
                self.integration_state = AIComponentState.INITIALIZING
                return
                
            # Perform AI coordination
            coordination_result = self.coordinate_ai_components()
            
            # Update integration state
            if coordination_result['confidence'] > self.confidence_threshold:
                self.integration_state = AIComponentState.READY
            elif coordination_result['confidence'] > 0.3:
                self.integration_state = AIComponentState.PROCESSING
            else:
                self.integration_state = AIComponentState.ERROR
                
            # Publish results
            self.publish_integration_results(coordination_result)
            
            self.last_integration_time = current_time
            
            # Log integration status
            self.get_logger().debug(f"AI Integration: {self.integration_state.value}, "
                                  f"Components: {coordination_result['healthy_components']}/{coordination_result['component_count']}, "
                                  f"Confidence: {coordination_result['confidence']:.2f}")

class PerceptionCoordinator:
    def __init__(self):
        self.alert_threshold = 0.8
        self.description_quality_threshold = 0.6
        
    def coordinate(self, perception_data):
        """Coordinate perception data"""
        result = {
            'status': 'ready',
            'confidence': 0.0,
            'alerts': [],
            'description_quality': 0.0
        }
        
        if 'description' in perception_data:
            result['description_quality'] = 0.8  # Simulated quality
            
        if 'alerts' in perception_data:
            try:
                alerts = json.loads(perception_data['alerts'].data)
                result['alerts'] = alerts
                if alerts:
                    result['confidence'] = 0.9
            except:
                result['confidence'] = 0.5
                
        return result

class DecisionCoordinator:
    def __init__(self):
        self.decision_confidence_threshold = 0.7
        
    def coordinate(self, decision_data):
        """Coordinate decision data"""
        result = {
            'status': 'ready',
            'confidence': 0.0,
            'mission_state': 'unknown',
            'decisions_made': 0
        }
        
        if 'status' in decision_data:
            try:
                status = json.loads(decision_data['status'].data)
                result['mission_state'] = status.get('mission_state', 'unknown')
                result['confidence'] = status.get('confidence', 0.0)
                result['decisions_made'] = 1
            except:
                result['confidence'] = 0.5
                
        return result

class SLAMCoordinator:
    def __init__(self):
        self.slam_confidence_threshold = 0.6
        
    def coordinate(self, slam_data):
        """Coordinate SLAM data"""
        result = {
            'status': 'ready',
            'confidence': 0.0,
            'pose_quality': 0.0,
            'map_quality': 0.0
        }
        
        if 'status' in slam_data:
            try:
                status = json.loads(slam_data['status'].data)
                result['confidence'] = status.get('confidence', 0.0)
                result['pose_quality'] = 0.8  # Simulated quality
                result['map_quality'] = 0.7   # Simulated quality
            except:
                result['confidence'] = 0.5
                
        return result

class FusionCoordinator:
    def __init__(self):
        self.fusion_confidence_threshold = 0.7
        
    def coordinate(self, fusion_data):
        """Coordinate fusion data"""
        result = {
            'status': 'ready',
            'confidence': 0.0,
            'sensor_count': 0,
            'fusion_quality': 0.0
        }
        
        if 'status' in fusion_data:
            try:
                status = json.loads(fusion_data['status'].data)
                result['confidence'] = status.get('confidence', 0.0)
                result['sensor_count'] = status.get('total_sensors', 0)
                result['fusion_quality'] = 0.8  # Simulated quality
            except:
                result['confidence'] = 0.5
                
        return result

class NavigationCoordinator:
    def __init__(self):
        self.navigation_confidence_threshold = 0.6
        
    def coordinate(self, navigation_data):
        """Coordinate navigation data"""
        result = {
            'status': 'ready',
            'confidence': 0.0,
            'path_quality': 0.0,
            'obstacle_avoidance': 0.0
        }
        
        if 'status' in navigation_data:
            try:
                status = json.loads(navigation_data['status'].data)
                result['confidence'] = 0.8  # Simulated confidence
                result['path_quality'] = 0.7  # Simulated quality
                result['obstacle_avoidance'] = 0.9  # Simulated quality
            except:
                result['confidence'] = 0.5
                
        return result

class SwarmCoordinator:
    def __init__(self):
        self.swarm_confidence_threshold = 0.7
        
    def coordinate(self, swarm_data):
        """Coordinate swarm data"""
        result = {
            'status': 'ready',
            'confidence': 0.0,
            'formation_quality': 0.0,
            'collision_avoidance': 0.0
        }
        
        if 'status' in swarm_data:
            try:
                status = json.loads(swarm_data['status'].data)
                result['confidence'] = 0.8  # Simulated confidence
                result['formation_quality'] = 0.8  # Simulated quality
                result['collision_avoidance'] = 0.9  # Simulated quality
            except:
                result['confidence'] = 0.5
                
        return result

def main(args=None):
    rclpy.init(args=args)
    ai_integration = RealTimeAIIntegration()
    
    try:
        rclpy.spin(ai_integration)
    except KeyboardInterrupt:
        ai_integration.get_logger().info("Real-time AI Integration stopped by user")
    finally:
        ai_integration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 