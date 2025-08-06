#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import cv2
import time
import random
from threading import Lock
import json

class VisionPLM(Node):
    def __init__(self):
        super().__init__('vision_plm')
        
        # PLM parameters
        self.description_frequency = 2.0  # Hz
        self.max_description_length = 200
        self.confidence_threshold = 0.7
        
        # AI model state (simulated)
        self.model_loaded = True
        self.processing_latency = 0.5  # seconds
        self.model_confidence = 0.85
        
        # Perception state
        self.plm_lock = Lock()
        self.current_environment = {}
        self.last_description_time = 0
        self.description_count = 0
        
        # Publishers
        self.description_pub = self.create_publisher(String, '/quadcopter/perception/description', 10)
        self.analysis_pub = self.create_publisher(String, '/quadcopter/perception/analysis', 10)
        self.alert_pub = self.create_publisher(String, '/quadcopter/perception/alerts', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/quadcopter/sensors/camera/rgb', 
            self.image_callback, 10
        )
        self.thermal_sub = self.create_subscription(
            Image, '/quadcopter/sensors/camera/thermal', 
            self.thermal_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/quadcopter/sensors/lidar/pointcloud', 
            self.pointcloud_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/quadcopter/slam/pose', 
            self.pose_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/quadcopter/slam/map', 
            self.map_callback, 10
        )
        
        # PLM timer
        self.plm_timer = self.create_timer(1.0/self.description_frequency, self.update_plm)
        
        self.get_logger().info("Vision PLM initialized")
        
    def image_callback(self, msg):
        """Process RGB camera data"""
        with self.plm_lock:
            # Simulate image analysis
            self.analyze_rgb_image(msg)
            
    def thermal_callback(self, msg):
        """Process thermal camera data"""
        with self.plm_lock:
            # Simulate thermal analysis
            self.analyze_thermal_image(msg)
            
    def pointcloud_callback(self, msg):
        """Process point cloud data"""
        with self.plm_lock:
            # Simulate point cloud analysis
            self.analyze_pointcloud(msg)
            
    def pose_callback(self, msg):
        """Process pose data"""
        with self.plm_lock:
            self.current_environment['pose'] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            }
            
    def map_callback(self, msg):
        """Process map data"""
        with self.plm_lock:
            # Simulate map analysis
            self.analyze_map(msg)
            
    def analyze_rgb_image(self, msg):
        """Analyze RGB image for objects and features"""
        # Simulate AI vision analysis
        height, width = msg.height, msg.width
        
        # Generate synthetic object detections
        objects = []
        for i in range(random.randint(1, 5)):
            obj = {
                'type': random.choice(['person', 'vehicle', 'building', 'equipment', 'obstacle']),
                'confidence': random.uniform(0.7, 0.95),
                'position': {
                    'x': random.uniform(0, width),
                    'y': random.uniform(0, height)
                },
                'size': {
                    'width': random.uniform(10, 100),
                    'height': random.uniform(10, 100)
                }
            }
            objects.append(obj)
            
        self.current_environment['rgb_objects'] = objects
        self.current_environment['rgb_timestamp'] = time.time()
        
    def analyze_thermal_image(self, msg):
        """Analyze thermal image for heat signatures"""
        # Simulate thermal analysis
        heat_signatures = []
        for i in range(random.randint(0, 3)):
            signature = {
                'type': random.choice(['human', 'vehicle', 'equipment', 'fire']),
                'temperature': random.uniform(20, 100),
                'confidence': random.uniform(0.6, 0.9),
                'position': {
                    'x': random.uniform(0, msg.width),
                    'y': random.uniform(0, msg.height)
                }
            }
            heat_signatures.append(signature)
            
        self.current_environment['thermal_signatures'] = heat_signatures
        self.current_environment['thermal_timestamp'] = time.time()
        
    def analyze_pointcloud(self, msg):
        """Analyze point cloud for 3D objects and obstacles"""
        # Simulate point cloud analysis
        obstacles = []
        for i in range(random.randint(2, 8)):
            obstacle = {
                'type': random.choice(['wall', 'door', 'window', 'furniture', 'equipment']),
                'distance': random.uniform(1, 20),
                'angle': random.uniform(0, 360),
                'height': random.uniform(0.5, 3.0),
                'confidence': random.uniform(0.7, 0.95)
            }
            obstacles.append(obstacle)
            
        self.current_environment['obstacles'] = obstacles
        self.current_environment['pointcloud_timestamp'] = time.time()
        
    def analyze_map(self, msg):
        """Analyze occupancy grid map"""
        # Simulate map analysis
        map_features = {
            'open_areas': random.randint(1, 5),
            'obstacles': random.randint(3, 10),
            'corridors': random.randint(1, 3),
            'rooms': random.randint(1, 4),
            'coverage_percentage': random.uniform(60, 95)
        }
        
        self.current_environment['map_features'] = map_features
        self.current_environment['map_timestamp'] = time.time()
        
    def generate_environment_description(self):
        """Generate natural language description of environment"""
        if not self.current_environment:
            return "No environment data available."
            
        # Simulate AI processing latency
        time.sleep(self.processing_latency)
        
        # Build description based on available data
        description_parts = []
        
        # Location description
        if 'pose' in self.current_environment:
            pose = self.current_environment['pose']
            description_parts.append(f"Quadcopter is positioned at coordinates ({pose['x']:.1f}, {pose['y']:.1f}, {pose['z']:.1f})")
            
        # Object detection
        if 'rgb_objects' in self.current_environment:
            objects = self.current_environment['rgb_objects']
            if objects:
                object_types = [obj['type'] for obj in objects]
                unique_types = list(set(object_types))
                description_parts.append(f"Visual sensors detect {len(objects)} objects including {', '.join(unique_types)}")
                
        # Thermal analysis
        if 'thermal_signatures' in self.current_environment:
            signatures = self.current_environment['thermal_signatures']
            if signatures:
                heat_types = [sig['type'] for sig in signatures]
                description_parts.append(f"Thermal sensors detect {len(signatures)} heat signatures: {', '.join(heat_types)}")
                
        # Obstacle analysis
        if 'obstacles' in self.current_environment:
            obstacles = self.current_environment['obstacles']
            if obstacles:
                avg_distance = np.mean([obs['distance'] for obs in obstacles])
                description_parts.append(f"Lidar detects {len(obstacles)} obstacles at average distance of {avg_distance:.1f} meters")
                
        # Map analysis
        if 'map_features' in self.current_environment:
            features = self.current_environment['map_features']
            description_parts.append(f"Map shows {features['open_areas']} open areas, {features['obstacles']} obstacles, "
                                  f"and {features['coverage_percentage']:.1f}% area coverage")
                                  
        # Combine description
        if description_parts:
            description = ". ".join(description_parts) + "."
        else:
            description = "Environment analysis in progress."
            
        return description
        
    def generate_situational_analysis(self):
        """Generate situational analysis and alerts"""
        analysis = {
            'timestamp': time.time(),
            'confidence': self.model_confidence,
            'alerts': [],
            'recommendations': []
        }
        
        # Check for potential hazards
        if 'obstacles' in self.current_environment:
            obstacles = self.current_environment['obstacles']
            close_obstacles = [obs for obs in obstacles if obs['distance'] < 3.0]
            
            if close_obstacles:
                analysis['alerts'].append({
                    'type': 'collision_risk',
                    'severity': 'medium',
                    'message': f"Detected {len(close_obstacles)} obstacles within 3 meters"
                })
                
        # Check for human presence
        if 'rgb_objects' in self.current_environment:
            objects = self.current_environment['rgb_objects']
            humans = [obj for obj in objects if obj['type'] == 'person']
            
            if humans:
                analysis['alerts'].append({
                    'type': 'human_detected',
                    'severity': 'high',
                    'message': f"Detected {len(humans)} person(s) in the environment"
                })
                
        # Check for thermal anomalies
        if 'thermal_signatures' in self.current_environment:
            signatures = self.current_environment['thermal_signatures']
            high_temp = [sig for sig in signatures if sig['temperature'] > 60]
            
            if high_temp:
                analysis['alerts'].append({
                    'type': 'high_temperature',
                    'severity': 'high',
                    'message': f"Detected {len(high_temp)} high-temperature signatures"
                })
                
        # Generate recommendations
        if len(analysis['alerts']) == 0:
            analysis['recommendations'].append("Environment appears safe for continued operation")
        else:
            analysis['recommendations'].append("Exercise caution and consider adjusting flight path")
            
        return analysis
        
    def publish_description(self, description):
        """Publish natural language description"""
        description_msg = String()
        description_msg.data = description
        self.description_pub.publish(description_msg)
        
    def publish_analysis(self, analysis):
        """Publish situational analysis"""
        analysis_msg = String()
        analysis_msg.data = json.dumps(analysis)
        self.analysis_pub.publish(analysis_msg)
        
    def publish_alerts(self, alerts):
        """Publish critical alerts"""
        if alerts:
            alert_msg = String()
            alert_msg.data = json.dumps(alerts)
            self.alert_pub.publish(alert_msg)
            
    def update_plm(self):
        """Main PLM update loop"""
        with self.plm_lock:
            current_time = time.time()
            
            # Check if we should generate new description
            if current_time - self.last_description_time > (1.0 / self.description_frequency):
                
                # Generate environment description
                description = self.generate_environment_description()
                self.publish_description(description)
                
                # Generate situational analysis
                analysis = self.generate_situational_analysis()
                self.publish_analysis(analysis)
                
                # Publish critical alerts
                critical_alerts = [alert for alert in analysis['alerts'] if alert['severity'] == 'high']
                self.publish_alerts(critical_alerts)
                
                self.last_description_time = current_time
                self.description_count += 1
                
                # Log description
                self.get_logger().info(f"PLM Description #{self.description_count}: {description[:100]}...")
                
                if critical_alerts:
                    self.get_logger().warn(f"🚨 {len(critical_alerts)} critical alerts generated")

def main(args=None):
    rclpy.init(args=args)
    vision_plm = VisionPLM()
    
    try:
        rclpy.spin(vision_plm)
    except KeyboardInterrupt:
        vision_plm.get_logger().info("Vision PLM stopped by user")
    finally:
        vision_plm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 