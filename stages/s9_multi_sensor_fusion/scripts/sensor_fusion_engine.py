#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
from threading import Lock
from enum import Enum
from collections import defaultdict
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class FusionState(Enum):
    INITIALIZING = "initializing"
    FUSING = "fusing"
    OPTIMIZING = "optimizing"
    PUBLISHING = "publishing"
    ERROR = "error"

class SensorType(Enum):
    CAMERA = "camera"
    LIDAR = "lidar"
    GPS = "gps"
    IMU = "imu"
    ULTRASONIC = "ultrasonic"
    THERMAL = "thermal"

class SensorFusionEngine(Node):
    def __init__(self):
        super().__init__('sensor_fusion_engine')
        
        # Fusion parameters
        self.fusion_frequency = 10.0  # Hz
        self.sensor_timeout = 1.0  # seconds
        self.confidence_threshold = 0.7
        self.max_sensor_delay = 0.5  # seconds
        
        # Fusion state
        self.fusion_state = FusionState.INITIALIZING
        self.sensor_data = {}
        self.fusion_results = {}
        self.sensor_weights = {}
        self.last_fusion_time = 0
        
        # Sensor tracking
        self.active_sensors = set()
        self.sensor_health = {}
        self.sensor_timestamps = {}
        
        # Thread safety
        self.fusion_lock = Lock()
        
        # CV Bridge for image processing
        self.cv_bridge = CvBridge()
        
        # Publishers
        self.fused_data_pub = self.create_publisher(String, '/quadcopter/fusion/data', 10)
        self.fusion_status_pub = self.create_publisher(String, '/quadcopter/fusion/status', 10)
        self.fusion_markers_pub = self.create_publisher(MarkerArray, '/quadcopter/fusion/markers', 10)
        
        # Subscribers for each sensor type
        self.setup_sensor_subscribers()
        
        # Fusion timer
        self.fusion_timer = self.create_timer(1.0/self.fusion_frequency, self.fusion_loop)
        
        # Fusion algorithms
        self.kalman_filter = KalmanFusion()
        self.particle_filter = ParticleFusion()
        self.ekf_filter = ExtendedKalmanFusion()
        
        self.get_logger().info("Sensor Fusion Engine initialized")
        
    def setup_sensor_subscribers(self):
        """Setup subscribers for all sensor types"""
        # Camera sensors
        self.rgb_camera_sub = self.create_subscription(
            Image, '/quadcopter/camera/rgb', 
            lambda msg: self.sensor_callback(SensorType.CAMERA, 'rgb', msg), 10
        )
        self.thermal_camera_sub = self.create_subscription(
            Image, '/quadcopter/camera/thermal', 
            lambda msg: self.sensor_callback(SensorType.THERMAL, 'thermal', msg), 10
        )
        
        # Lidar sensors
        self.lidar_pointcloud_sub = self.create_subscription(
            PointCloud2, '/quadcopter/lidar/points', 
            lambda msg: self.sensor_callback(SensorType.LIDAR, 'pointcloud', msg), 10
        )
        self.lidar_scan_sub = self.create_subscription(
            LaserScan, '/quadcopter/lidar/scan', 
            lambda msg: self.sensor_callback(SensorType.LIDAR, 'scan', msg), 10
        )
        
        # Navigation sensors
        self.gps_sub = self.create_subscription(
            NavSatFix, '/quadcopter/gps', 
            lambda msg: self.sensor_callback(SensorType.GPS, 'gps', msg), 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/quadcopter/imu', 
            lambda msg: self.sensor_callback(SensorType.IMU, 'imu', msg), 10
        )
        
        # Ultrasonic sensors
        self.ultrasonic_sub = self.create_subscription(
            LaserScan, '/quadcopter/ultrasonic', 
            lambda msg: self.sensor_callback(SensorType.ULTRASONIC, 'ultrasonic', msg), 10
        )
        
    def sensor_callback(self, sensor_type, sensor_name, msg):
        """Process sensor data"""
        with self.fusion_lock:
            current_time = time.time()
            
            # Update sensor health
            sensor_key = f"{sensor_type.value}_{sensor_name}"
            self.active_sensors.add(sensor_key)
            self.sensor_health[sensor_key] = True
            self.sensor_timestamps[sensor_key] = current_time
            
            # Store sensor data
            if sensor_type not in self.sensor_data:
                self.sensor_data[sensor_type] = {}
                
            self.sensor_data[sensor_type][sensor_name] = {
                'data': msg,
                'timestamp': current_time,
                'processed': False
            }
            
    def process_camera_data(self, sensor_data):
        """Process camera sensor data"""
        processed_data = {}
        
        for camera_type, data in sensor_data.items():
            try:
                # Convert ROS image to OpenCV
                cv_image = self.cv_bridge.imgmsg_to_cv2(data['data'], "bgr8")
                
                # Basic image processing
                processed_data[camera_type] = {
                    'image': cv_image,
                    'shape': cv_image.shape,
                    'timestamp': data['timestamp'],
                    'features': self.extract_image_features(cv_image)
                }
                
            except Exception as e:
                self.get_logger().warn(f"Error processing {camera_type} camera: {e}")
                
        return processed_data
        
    def process_lidar_data(self, sensor_data):
        """Process lidar sensor data"""
        processed_data = {}
        
        for lidar_type, data in sensor_data.items():
            try:
                if lidar_type == 'pointcloud':
                    # Process point cloud data
                    processed_data[lidar_type] = {
                        'point_count': len(data['data'].data),
                        'timestamp': data['timestamp'],
                        'features': self.extract_pointcloud_features(data['data'])
                    }
                elif lidar_type == 'scan':
                    # Process laser scan data
                    scan_data = data['data']
                    processed_data[lidar_type] = {
                        'ranges': scan_data.ranges,
                        'angles': np.linspace(scan_data.angle_min, scan_data.angle_max, len(scan_data.ranges)),
                        'timestamp': data['timestamp'],
                        'features': self.extract_scan_features(scan_data)
                    }
                    
            except Exception as e:
                self.get_logger().warn(f"Error processing {lidar_type} lidar: {e}")
                
        return processed_data
        
    def process_navigation_data(self, sensor_data):
        """Process navigation sensor data"""
        processed_data = {}
        
        for nav_type, data in sensor_data.items():
            try:
                if nav_type == 'gps':
                    gps_data = data['data']
                    processed_data[nav_type] = {
                        'latitude': gps_data.latitude,
                        'longitude': gps_data.longitude,
                        'altitude': gps_data.altitude,
                        'timestamp': data['timestamp']
                    }
                elif nav_type == 'imu':
                    imu_data = data['data']
                    processed_data[nav_type] = {
                        'orientation': imu_data.orientation,
                        'angular_velocity': imu_data.angular_velocity,
                        'linear_acceleration': imu_data.linear_acceleration,
                        'timestamp': data['timestamp']
                    }
                    
            except Exception as e:
                self.get_logger().warn(f"Error processing {nav_type} navigation: {e}")
                
        return processed_data
        
    def extract_image_features(self, image):
        """Extract features from camera image"""
        features = {}
        
        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        features['edge_density'] = np.sum(edges > 0) / edges.size
        
        # Corner detection
        corners = cv2.goodFeaturesToTrack(gray, 25, 0.01, 10)
        features['corner_count'] = len(corners) if corners is not None else 0
        
        # Brightness analysis
        features['brightness'] = np.mean(gray)
        features['contrast'] = np.std(gray)
        
        return features
        
    def extract_pointcloud_features(self, pointcloud):
        """Extract features from point cloud"""
        features = {}
        
        # Basic point cloud statistics
        features['point_count'] = len(pointcloud.data)
        features['density'] = features['point_count'] / 1000.0  # Normalized density
        
        return features
        
    def extract_scan_features(self, scan_data):
        """Extract features from laser scan"""
        features = {}
        
        # Range statistics
        valid_ranges = [r for r in scan_data.ranges if r > scan_data.range_min and r < scan_data.range_max]
        features['avg_range'] = np.mean(valid_ranges) if valid_ranges else 0
        features['min_range'] = np.min(valid_ranges) if valid_ranges else 0
        features['max_range'] = np.max(valid_ranges) if valid_ranges else 0
        features['range_variance'] = np.var(valid_ranges) if valid_ranges else 0
        
        return features
        
    def check_sensor_health(self):
        """Check health of all sensors"""
        current_time = time.time()
        healthy_sensors = 0
        total_sensors = len(self.active_sensors)
        
        for sensor_key in self.active_sensors:
            if sensor_key in self.sensor_timestamps:
                time_since_update = current_time - self.sensor_timestamps[sensor_key]
                if time_since_update < self.sensor_timeout:
                    healthy_sensors += 1
                    self.sensor_health[sensor_key] = True
                else:
                    self.sensor_health[sensor_key] = False
                    
        return healthy_sensors, total_sensors
        
    def fuse_sensor_data(self):
        """Fuse data from all sensors"""
        fusion_result = {
            'timestamp': time.time(),
            'sensor_count': len(self.active_sensors),
            'healthy_sensors': 0,
            'fused_data': {},
            'confidence': 0.0,
            'alerts': []
        }
        
        # Process each sensor type
        processed_data = {}
        
        # Process camera data
        if SensorType.CAMERA in self.sensor_data:
            processed_data['camera'] = self.process_camera_data(self.sensor_data[SensorType.CAMERA])
            
        # Process lidar data
        if SensorType.LIDAR in self.sensor_data:
            processed_data['lidar'] = self.process_lidar_data(self.sensor_data[SensorType.LIDAR])
            
        # Process navigation data
        if SensorType.GPS in self.sensor_data or SensorType.IMU in self.sensor_data:
            nav_data = {}
            if SensorType.GPS in self.sensor_data:
                nav_data.update(self.sensor_data[SensorType.GPS])
            if SensorType.IMU in self.sensor_data:
                nav_data.update(self.sensor_data[SensorType.IMU])
            processed_data['navigation'] = self.process_navigation_data(nav_data)
            
        # Apply fusion algorithms
        fusion_result['fused_data'] = self.apply_fusion_algorithms(processed_data)
        
        # Calculate confidence
        healthy_sensors, total_sensors = self.check_sensor_health()
        fusion_result['healthy_sensors'] = healthy_sensors
        fusion_result['confidence'] = healthy_sensors / max(total_sensors, 1)
        
        # Generate alerts
        if fusion_result['confidence'] < self.confidence_threshold:
            fusion_result['alerts'].append("Low sensor confidence - check sensor health")
            
        return fusion_result
        
    def apply_fusion_algorithms(self, processed_data):
        """Apply different fusion algorithms"""
        fused_data = {}
        
        # Kalman filter fusion
        if 'navigation' in processed_data:
            kalman_result = self.kalman_filter.fuse(processed_data['navigation'])
            fused_data['kalman'] = kalman_result
            
        # Particle filter fusion
        if 'lidar' in processed_data and 'camera' in processed_data:
            particle_result = self.particle_filter.fuse(processed_data['lidar'], processed_data['camera'])
            fused_data['particle'] = particle_result
            
        # Extended Kalman filter fusion
        if len(processed_data) > 1:
            ekf_result = self.ekf_filter.fuse(processed_data)
            fused_data['ekf'] = ekf_result
            
        return fused_data
        
    def publish_fusion_results(self, fusion_result):
        """Publish fusion results"""
        # Publish fused data
        fused_data_msg = String()
        fused_data_msg.data = json.dumps(fusion_result)
        self.fused_data_pub.publish(fused_data_msg)
        
        # Publish fusion status
        status_msg = String()
        status_data = {
            'fusion_state': self.fusion_state.value,
            'healthy_sensors': fusion_result['healthy_sensors'],
            'total_sensors': fusion_result['sensor_count'],
            'confidence': fusion_result['confidence'],
            'alerts': fusion_result['alerts']
        }
        status_msg.data = json.dumps(status_data)
        self.fusion_status_pub.publish(status_msg)
        
        # Publish fusion markers
        self.publish_fusion_markers(fusion_result)
        
    def publish_fusion_markers(self, fusion_result):
        """Publish fusion visualization markers"""
        marker_array = MarkerArray()
        
        # Sensor health markers
        for i, (sensor_key, is_healthy) in enumerate(self.sensor_health.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sensor_health"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = i * 2.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
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
            
        self.fusion_markers_pub.publish(marker_array)
        
    def fusion_loop(self):
        """Main fusion loop"""
        with self.fusion_lock:
            current_time = time.time()
            
            # Check if we have enough sensor data
            if len(self.active_sensors) < 2:
                self.fusion_state = FusionState.INITIALIZING
                return
                
            # Perform sensor fusion
            fusion_result = self.fuse_sensor_data()
            
            # Update fusion state
            if fusion_result['confidence'] > self.confidence_threshold:
                self.fusion_state = FusionState.FUSING
            else:
                self.fusion_state = FusionState.ERROR
                
            # Publish results
            self.publish_fusion_results(fusion_result)
            
            self.last_fusion_time = current_time
            
            # Log fusion status
            self.get_logger().debug(f"Fusion: {self.fusion_state.value}, "
                                  f"Sensors: {fusion_result['healthy_sensors']}/{fusion_result['sensor_count']}, "
                                  f"Confidence: {fusion_result['confidence']:.2f}")

class KalmanFusion:
    def __init__(self):
        self.state = np.zeros(6)  # [x, y, z, vx, vy, vz]
        self.covariance = np.eye(6) * 0.1
        
    def fuse(self, navigation_data):
        """Fuse navigation data using Kalman filter"""
        result = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'confidence': 0.8
        }
        
        # Simple Kalman filter implementation
        if 'gps' in navigation_data:
            gps = navigation_data['gps']
            # Convert GPS to local coordinates (simplified)
            result['position']['x'] = gps['longitude'] * 111000  # Approximate conversion
            result['position']['y'] = gps['latitude'] * 111000
            result['position']['z'] = gps['altitude']
            
        if 'imu' in navigation_data:
            imu = navigation_data['imu']
            # Extract velocity from IMU
            result['velocity']['x'] = imu['linear_acceleration'].x
            result['velocity']['y'] = imu['linear_acceleration'].y
            result['velocity']['z'] = imu['linear_acceleration'].z
            
        return result

class ParticleFusion:
    def __init__(self):
        self.particles = []
        self.num_particles = 100
        
    def fuse(self, lidar_data, camera_data):
        """Fuse lidar and camera data using particle filter"""
        result = {
            'objects': [],
            'obstacles': [],
            'confidence': 0.7
        }
        
        # Simulate object detection from camera
        if 'rgb' in camera_data:
            features = camera_data['rgb']['features']
            if features['corner_count'] > 10:
                result['objects'].append({
                    'type': 'unknown',
                    'position': {'x': 5.0, 'y': 0.0, 'z': 0.0},
                    'confidence': 0.6
                })
                
        # Simulate obstacle detection from lidar
        if 'scan' in lidar_data:
            scan_features = lidar_data['scan']['features']
            if scan_features['min_range'] < 3.0:
                result['obstacles'].append({
                    'distance': scan_features['min_range'],
                    'angle': 0.0,
                    'confidence': 0.8
                })
                
        return result

class ExtendedKalmanFusion:
    def __init__(self):
        self.state = np.zeros(9)  # [x, y, z, vx, vy, vz, ax, ay, az]
        self.covariance = np.eye(9) * 0.1
        
    def fuse(self, all_sensor_data):
        """Fuse all sensor data using Extended Kalman Filter"""
        result = {
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'confidence': 0.9
        }
        
        # EKF fusion implementation
        if 'navigation' in all_sensor_data:
            nav_data = all_sensor_data['navigation']
            if 'gps' in nav_data:
                gps = nav_data['gps']
                result['pose']['x'] = gps['longitude'] * 111000
                result['pose']['y'] = gps['latitude'] * 111000
                result['pose']['z'] = gps['altitude']
                
            if 'imu' in nav_data:
                imu = nav_data['imu']
                result['velocity']['x'] = imu['linear_acceleration'].x
                result['velocity']['y'] = imu['linear_acceleration'].y
                result['velocity']['z'] = imu['linear_acceleration'].z
                result['acceleration']['x'] = imu['linear_acceleration'].x
                result['acceleration']['y'] = imu['linear_acceleration'].y
                result['acceleration']['z'] = imu['linear_acceleration'].z
                
        return result

def main(args=None):
    rclpy.init(args=args)
    fusion_engine = SensorFusionEngine()
    
    try:
        rclpy.spin(fusion_engine)
    except KeyboardInterrupt:
        fusion_engine.get_logger().info("Sensor Fusion Engine stopped by user")
    finally:
        fusion_engine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 