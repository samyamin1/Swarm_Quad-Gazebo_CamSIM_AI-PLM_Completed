#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
from threading import Lock
from enum import Enum
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class SLAMState(Enum):
    INITIALIZING = "initializing"
    MAPPING = "mapping"
    LOCALIZING = "localizing"
    LOOP_CLOSURE = "loop_closure"
    OPTIMIZING = "optimizing"
    ERROR = "error"

class SLAMType(Enum):
    VISUAL = "visual"
    LIDAR = "lidar"
    HYBRID = "hybrid"

class AdvancedSLAMEngine(Node):
    def __init__(self):
        super().__init__('advanced_slam_engine')
        
        # SLAM parameters
        self.slam_frequency = 10.0  # Hz
        self.map_resolution = 0.05  # meters
        self.map_width = 1000  # cells
        self.map_height = 1000  # cells
        self.max_range = 10.0  # meters
        self.min_range = 0.1  # meters
        
        # SLAM state
        self.slam_state = SLAMState.INITIALIZING
        self.slam_type = SLAMType.HYBRID
        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.initial_pose = None
        self.pose_history = []
        self.keyframes = []
        
        # Mapping data
        self.occupancy_grid = None
        self.map_origin = {'x': -25.0, 'y': -25.0, 'z': 0.0}
        self.map_updated = False
        
        # Thread safety
        self.slam_lock = Lock()
        
        # CV Bridge for image processing
        self.cv_bridge = CvBridge()
        
        # TF2 for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/quadcopter/slam/pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/quadcopter/slam/map', 10)
        self.path_pub = self.create_publisher(Path, '/quadcopter/slam/path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/quadcopter/slam/markers', 10)
        self.slam_status_pub = self.create_publisher(String, '/quadcopter/slam/status', 10)
        
        # Subscribers
        self.setup_slam_subscribers()
        
        # SLAM timer
        self.slam_timer = self.create_timer(1.0/self.slam_frequency, self.slam_loop)
        
        # SLAM algorithms
        self.visual_slam = VisualSLAM()
        self.lidar_slam = LidarSLAM()
        self.hybrid_slam = HybridSLAM()
        
        # Initialize occupancy grid
        self.initialize_occupancy_grid()
        
        self.get_logger().info("Advanced SLAM Engine initialized")
        
    def setup_slam_subscribers(self):
        """Setup subscribers for SLAM data"""
        # Visual SLAM inputs
        self.rgb_camera_sub = self.create_subscription(
            Image, '/quadcopter/camera/rgb', 
            self.rgb_camera_callback, 10
        )
        self.depth_camera_sub = self.create_subscription(
            Image, '/quadcopter/camera/depth', 
            self.depth_camera_callback, 10
        )
        
        # Lidar SLAM inputs
        self.lidar_pointcloud_sub = self.create_subscription(
            PointCloud2, '/quadcopter/lidar/points', 
            self.lidar_pointcloud_callback, 10
        )
        self.lidar_scan_sub = self.create_subscription(
            LaserScan, '/quadcopter/lidar/scan', 
            self.lidar_scan_callback, 10
        )
        
        # IMU for odometry
        self.imu_sub = self.create_subscription(
            Imu, '/quadcopter/imu', 
            self.imu_callback, 10
        )
        
    def rgb_camera_callback(self, msg):
        """Process RGB camera data for visual SLAM"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.visual_slam.process_image(cv_image, time.time())
        except Exception as e:
            self.get_logger().warn(f"Error processing RGB camera: {e}")
            
    def depth_camera_callback(self, msg):
        """Process depth camera data for visual SLAM"""
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "16UC1")
            self.visual_slam.process_depth(depth_image, time.time())
        except Exception as e:
            self.get_logger().warn(f"Error processing depth camera: {e}")
            
    def lidar_pointcloud_callback(self, msg):
        """Process lidar point cloud data"""
        try:
            self.lidar_slam.process_pointcloud(msg, time.time())
        except Exception as e:
            self.get_logger().warn(f"Error processing lidar pointcloud: {e}")
            
    def lidar_scan_callback(self, msg):
        """Process lidar scan data"""
        try:
            self.lidar_slam.process_scan(msg, time.time())
        except Exception as e:
            self.get_logger().warn(f"Error processing lidar scan: {e}")
            
    def imu_callback(self, msg):
        """Process IMU data for odometry"""
        try:
            # Extract orientation and acceleration
            orientation = msg.orientation
            linear_acc = msg.linear_acceleration
            angular_vel = msg.angular_velocity
            
            # Update current pose with IMU data
            self.update_pose_from_imu(orientation, linear_acc, angular_vel)
        except Exception as e:
            self.get_logger().warn(f"Error processing IMU: {e}")
            
    def initialize_occupancy_grid(self):
        """Initialize the occupancy grid map"""
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = "map"
        self.occupancy_grid.info.resolution = self.map_resolution
        self.occupancy_grid.info.width = self.map_width
        self.occupancy_grid.info.height = self.map_height
        self.occupancy_grid.info.origin.position.x = self.map_origin['x']
        self.occupancy_grid.info.origin.position.y = self.map_origin['y']
        self.occupancy_grid.info.origin.position.z = self.map_origin['z']
        self.occupancy_grid.info.origin.orientation.w = 1.0
        
        # Initialize with unknown cells (-1)
        self.occupancy_grid.data = [-1] * (self.map_width * self.map_height)
        
    def update_pose_from_imu(self, orientation, linear_acc, angular_vel):
        """Update pose using IMU data"""
        # Simple IMU integration (in real implementation, use proper sensor fusion)
        dt = 0.1  # 10Hz update rate
        
        # Extract yaw from quaternion
        yaw = np.arctan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                          1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
        
        # Update current pose
        self.current_pose['yaw'] = yaw
        
        # Simple velocity integration
        if hasattr(self, 'last_linear_acc'):
            velocity_x = (linear_acc.x + self.last_linear_acc.x) * dt / 2.0
            velocity_y = (linear_acc.y + self.last_linear_acc.y) * dt / 2.0
            velocity_z = (linear_acc.z + self.last_linear_acc.z) * dt / 2.0
            
            self.current_pose['x'] += velocity_x * dt
            self.current_pose['y'] += velocity_y * dt
            self.current_pose['z'] += velocity_z * dt
            
        self.last_linear_acc = linear_acc
        
    def update_occupancy_grid(self, scan_data):
        """Update occupancy grid with lidar scan data"""
        if self.occupancy_grid is None:
            return
            
        # Convert scan data to grid coordinates
        for i, range_reading in enumerate(scan_data.ranges):
            if range_reading < scan_data.range_min or range_reading > scan_data.range_max:
                continue
                
            # Calculate angle
            angle = scan_data.angle_min + i * scan_data.angle_increment
            
            # Calculate endpoint in world coordinates
            end_x = self.current_pose['x'] + range_reading * np.cos(angle + self.current_pose['yaw'])
            end_y = self.current_pose['y'] + range_reading * np.sin(angle + self.current_pose['yaw'])
            
            # Convert to grid coordinates
            grid_x = int((end_x - self.map_origin['x']) / self.map_resolution)
            grid_y = int((end_y - self.map_origin['y']) / self.map_resolution)
            
            # Check bounds
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                # Mark as occupied
                index = grid_y * self.map_width + grid_x
                self.occupancy_grid.data[index] = 100
                
                # Mark cells along the ray as free
                start_x = int((self.current_pose['x'] - self.map_origin['x']) / self.map_resolution)
                start_y = int((self.current_pose['y'] - self.map_origin['y']) / self.map_resolution)
                
                # Bresenham's line algorithm to mark free cells
                self.mark_free_cells(start_x, start_y, grid_x, grid_y)
                
    def mark_free_cells(self, x0, y0, x1, y1):
        """Mark cells along a line as free using Bresenham's algorithm"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                index = y * self.map_width + x
                if self.occupancy_grid.data[index] == -1:  # Unknown
                    self.occupancy_grid.data[index] = 0  # Free
                    
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
    def perform_slam(self):
        """Perform SLAM based on current type"""
        slam_result = {
            'timestamp': time.time(),
            'slam_type': self.slam_type.value,
            'slam_state': self.slam_state.value,
            'pose': self.current_pose.copy(),
            'confidence': 0.0,
            'keyframes': len(self.keyframes),
            'map_updated': self.map_updated
        }
        
        if self.slam_type == SLAMType.VISUAL:
            slam_result.update(self.visual_slam.get_result())
        elif self.slam_type == SLAMType.LIDAR:
            slam_result.update(self.lidar_slam.get_result())
        elif self.slam_type == SLAMType.HYBRID:
            slam_result.update(self.hybrid_slam.get_result())
            
        return slam_result
        
    def publish_slam_results(self, slam_result):
        """Publish SLAM results"""
        # Publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = slam_result['pose']['x']
        pose_msg.pose.position.y = slam_result['pose']['y']
        pose_msg.pose.position.z = slam_result['pose']['z']
        
        # Convert yaw to quaternion
        yaw = slam_result['pose']['yaw']
        pose_msg.pose.orientation.z = np.sin(yaw / 2.0)
        pose_msg.pose.orientation.w = np.cos(yaw / 2.0)
        
        self.pose_pub.publish(pose_msg)
        
        # Publish occupancy grid if updated
        if self.map_updated:
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.occupancy_grid)
            self.map_updated = False
            
        # Publish path
        if len(self.pose_history) > 1:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            
            for pose in self.pose_history[-100:]:  # Last 100 poses
                path_pose = PoseStamped()
                path_pose.header = path_msg.header
                path_pose.pose.position.x = pose['x']
                path_pose.pose.position.y = pose['y']
                path_pose.pose.position.z = pose['z']
                path_pose.pose.orientation.z = np.sin(pose['yaw'] / 2.0)
                path_pose.pose.orientation.w = np.cos(pose['yaw'] / 2.0)
                path_msg.poses.append(path_pose)
                
            self.path_pub.publish(path_msg)
            
        # Publish SLAM status
        status_msg = String()
        status_msg.data = json.dumps(slam_result)
        self.slam_status_pub.publish(status_msg)
        
        # Publish visualization markers
        self.publish_slam_markers(slam_result)
        
    def publish_slam_markers(self, slam_result):
        """Publish SLAM visualization markers"""
        marker_array = MarkerArray()
        
        # Current pose marker
        pose_marker = Marker()
        pose_marker.header.frame_id = "map"
        pose_marker.header.stamp = self.get_clock().now().to_msg()
        pose_marker.ns = "slam_pose"
        pose_marker.id = 0
        pose_marker.type = Marker.ARROW
        pose_marker.action = Marker.ADD
        
        pose_marker.pose.position.x = slam_result['pose']['x']
        pose_marker.pose.position.y = slam_result['pose']['y']
        pose_marker.pose.position.z = slam_result['pose']['z']
        pose_marker.pose.orientation.z = np.sin(slam_result['pose']['yaw'] / 2.0)
        pose_marker.pose.orientation.w = np.cos(slam_result['pose']['yaw'] / 2.0)
        
        pose_marker.scale.x = 1.0
        pose_marker.scale.y = 0.2
        pose_marker.scale.z = 0.2
        
        pose_marker.color.r = 0.0
        pose_marker.color.g = 1.0
        pose_marker.color.b = 0.0
        pose_marker.color.a = 0.8
        
        marker_array.markers.append(pose_marker)
        
        # Keyframe markers
        for i, keyframe in enumerate(self.keyframes[-10:]):  # Last 10 keyframes
            kf_marker = Marker()
            kf_marker.header.frame_id = "map"
            kf_marker.header.stamp = self.get_clock().now().to_msg()
            kf_marker.ns = "keyframes"
            kf_marker.id = i + 1
            kf_marker.type = Marker.SPHERE
            kf_marker.action = Marker.ADD
            
            kf_marker.pose.position.x = keyframe['x']
            kf_marker.pose.position.y = keyframe['y']
            kf_marker.pose.position.z = keyframe['z']
            
            kf_marker.scale.x = 0.3
            kf_marker.scale.y = 0.3
            kf_marker.scale.z = 0.3
            
            kf_marker.color.r = 1.0
            kf_marker.color.g = 0.0
            kf_marker.color.b = 0.0
            kf_marker.color.a = 0.6
            
            marker_array.markers.append(kf_marker)
            
        self.markers_pub.publish(marker_array)
        
    def slam_loop(self):
        """Main SLAM loop"""
        with self.slam_lock:
            current_time = time.time()
            
            # Update SLAM state
            if self.slam_state == SLAMState.INITIALIZING:
                if len(self.pose_history) > 5:
                    self.slam_state = SLAMState.MAPPING
                    
            # Perform SLAM
            slam_result = self.perform_slam()
            
            # Update pose history
            self.pose_history.append(slam_result['pose'].copy())
            if len(self.pose_history) > 1000:
                self.pose_history = self.pose_history[-1000:]
                
            # Add keyframe if significant movement
            if len(self.pose_history) > 1:
                last_pose = self.pose_history[-2]
                current_pose = self.pose_history[-1]
                distance = np.sqrt((current_pose['x'] - last_pose['x'])**2 + 
                                 (current_pose['y'] - last_pose['y'])**2)
                if distance > 0.5:  # 0.5m threshold
                    self.keyframes.append(current_pose.copy())
                    if len(self.keyframes) > 100:
                        self.keyframes = self.keyframes[-100:]
                        
            # Publish results
            self.publish_slam_results(slam_result)
            
            # Log SLAM status
            self.get_logger().debug(f"SLAM: {self.slam_state.value}, "
                                  f"Type: {self.slam_type.value}, "
                                  f"Pose: ({slam_result['pose']['x']:.2f}, {slam_result['pose']['y']:.2f}), "
                                  f"Confidence: {slam_result['confidence']:.2f}")

class VisualSLAM:
    def __init__(self):
        self.features = []
        self.descriptors = []
        self.keyframes = []
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
    def process_image(self, image, timestamp):
        """Process RGB image for visual SLAM"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect ORB features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        if keypoints is not None and descriptors is not None:
            self.features = keypoints
            self.descriptors = descriptors
            
    def process_depth(self, depth_image, timestamp):
        """Process depth image for visual SLAM"""
        # Process depth data for 3D reconstruction
        pass
        
    def get_result(self):
        """Get visual SLAM result"""
        return {
            'visual_features': len(self.features),
            'visual_descriptors': len(self.descriptors) if self.descriptors is not None else 0,
            'visual_keyframes': len(self.keyframes)
        }

class LidarSLAM:
    def __init__(self):
        self.scan_data = []
        self.pointcloud_data = []
        self.scan_processing = False
        
    def process_scan(self, scan_msg, timestamp):
        """Process laser scan data"""
        self.scan_data.append({
            'ranges': scan_msg.ranges,
            'angles': np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges)),
            'timestamp': timestamp
        })
        
        # Keep only recent scans
        if len(self.scan_data) > 100:
            self.scan_data = self.scan_data[-100:]
            
    def process_pointcloud(self, pointcloud_msg, timestamp):
        """Process point cloud data"""
        self.pointcloud_data.append({
            'data': pointcloud_msg,
            'timestamp': timestamp
        })
        
        # Keep only recent point clouds
        if len(self.pointcloud_data) > 50:
            self.pointcloud_data = self.pointcloud_data[-50:]
            
    def get_result(self):
        """Get lidar SLAM result"""
        return {
            'lidar_scans': len(self.scan_data),
            'lidar_pointclouds': len(self.pointcloud_data)
        }

class HybridSLAM:
    def __init__(self):
        self.visual_weight = 0.6
        self.lidar_weight = 0.4
        self.fusion_confidence = 0.8
        
    def get_result(self):
        """Get hybrid SLAM result"""
        return {
            'hybrid_confidence': self.fusion_confidence,
            'visual_weight': self.visual_weight,
            'lidar_weight': self.lidar_weight
        }

def main(args=None):
    rclpy.init(args=args)
    slam_engine = AdvancedSLAMEngine()
    
    try:
        rclpy.spin(slam_engine)
    except KeyboardInterrupt:
        slam_engine.get_logger().info("Advanced SLAM Engine stopped by user")
    finally:
        slam_engine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 