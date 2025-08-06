#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
import numpy as np
import cv2
import time
import random
from threading import Lock
from collections import deque

class VisualSLAM(Node):
    def __init__(self):
        super().__init__('visual_slam')
        
        # SLAM parameters
        self.min_features = 50
        self.max_features = 2000
        self.feature_quality = 0.01
        self.min_distance = 10.0
        
        # Tracking parameters
        self.max_tracking_points = 100
        self.tracking_threshold = 0.8
        self.ransac_threshold = 3.0
        
        # Mapping parameters
        self.map_resolution = 0.1  # meters per pixel
        self.map_size = 100  # meters
        self.map_pixels = int(self.map_size / self.map_resolution)
        
        # SLAM state
        self.slam_lock = Lock()
        self.is_initialized = False
        self.current_pose = None
        self.keyframes = []
        self.tracking_points = []
        self.map_points = []
        self.loop_detected = False
        
        # Feature detector
        self.orb = cv2.ORB_create(
            nfeatures=self.max_features,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            firstLevel=0,
            WTA_K=2,
            patchSize=31,
            fastThreshold=20
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/quadcopter/slam/pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/quadcopter/slam/map', 10)
        self.path_pub = self.create_publisher(Path, '/quadcopter/slam/path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/quadcopter/slam/markers', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/quadcopter/sensors/camera/rgb', 
            self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/quadcopter/sensors/camera/info', 
            self.camera_info_callback, 10
        )
        
        # SLAM timer
        self.slam_timer = self.create_timer(0.1, self.update_slam)
        
        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Path tracking
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        # Map
        self.occupancy_map = np.zeros((self.map_pixels, self.map_pixels), dtype=np.uint8)
        self.map_origin = np.array([-self.map_size/2, -self.map_size/2])
        
        self.get_logger().info("Visual SLAM initialized")
        
    def camera_info_callback(self, msg):
        """Process camera calibration info"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera calibration loaded")
            
    def image_callback(self, msg):
        """Process incoming camera images"""
        with self.slam_lock:
            # Convert ROS image to OpenCV format
            try:
                # For simplicity, create synthetic image data
                height, width = msg.height, msg.width
                image = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
                
                # Add synthetic features
                self.add_synthetic_features(image)
                
                # Process the image
                self.process_image(image)
                
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")
                
    def add_synthetic_features(self, image):
        """Add synthetic features to the image for testing"""
        height, width = image.shape[:2]
        
        # Add some synthetic corners and edges
        for i in range(20):
            x = random.randint(0, width-1)
            y = random.randint(0, height-1)
            size = random.randint(5, 15)
            color = [random.randint(100, 255)] * 3
            cv2.circle(image, (x, y), size, color, -1)
            
        # Add some lines (edges)
        for i in range(10):
            x1 = random.randint(0, width-1)
            y1 = random.randint(0, height-1)
            x2 = random.randint(0, width-1)
            y2 = random.randint(0, height-1)
            color = [random.randint(50, 150)] * 3
            cv2.line(image, (x1, y1), (x2, y2), color, 2)
            
    def process_image(self, image):
        """Process image for SLAM"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect ORB features
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        if keypoints is None or len(keypoints) < self.min_features:
            self.get_logger().warn("Insufficient features detected")
            return
            
        # Filter keypoints by quality
        good_keypoints = []
        good_descriptors = []
        
        for kp, desc in zip(keypoints, descriptors):
            if kp.response > self.feature_quality:
                good_keypoints.append(kp)
                good_descriptors.append(desc)
                
        if len(good_keypoints) < self.min_features:
            self.get_logger().warn("Insufficient good features")
            return
            
        # Limit number of features
        if len(good_keypoints) > self.max_features:
            indices = np.argsort([kp.response for kp in good_keypoints])[-self.max_features:]
            good_keypoints = [good_keypoints[i] for i in indices]
            good_descriptors = [good_descriptors[i] for i in indices]
            
        # Track features if we have previous frame
        if self.is_initialized and len(self.tracking_points) > 0:
            self.track_features(good_keypoints, good_descriptors)
        else:
            # Initialize tracking
            self.initialize_tracking(good_keypoints, good_descriptors)
            
        # Update keyframes
        self.update_keyframes(good_keypoints, good_descriptors)
        
    def initialize_tracking(self, keypoints, descriptors):
        """Initialize feature tracking"""
        self.tracking_points = keypoints[:self.max_tracking_points]
        self.tracking_descriptors = descriptors[:self.max_tracking_points]
        self.is_initialized = True
        self.get_logger().info("Feature tracking initialized")
        
    def track_features(self, keypoints, descriptors):
        """Track features between frames"""
        if len(self.tracking_points) == 0:
            return
            
        # Match features using brute force
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(self.tracking_descriptors, descriptors)
        
        # Filter good matches
        good_matches = [m for m in matches if m.distance < 50]
        
        if len(good_matches) < 10:
            self.get_logger().warn("Insufficient matches for tracking")
            return
            
        # Extract matched points
        src_pts = np.float32([self.tracking_points[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        
        # Estimate fundamental matrix
        F, mask = cv2.findFundamentalMat(src_pts, dst_pts, cv2.FM_RANSAC, 
                                        self.ransac_threshold, 0.99)
        
        if F is None:
            self.get_logger().warn("Could not estimate fundamental matrix")
            return
            
        # Filter inliers
        inliers = mask.ravel().astype(bool)
        good_matches = [m for i, m in enumerate(good_matches) if inliers[i]]
        
        if len(good_matches) < 8:
            self.get_logger().warn("Insufficient inliers for pose estimation")
            return
            
        # Estimate pose (simplified)
        self.estimate_pose(src_pts[inliers], dst_pts[inliers])
        
        # Update tracking points
        self.tracking_points = [keypoints[m.trainIdx] for m in good_matches]
        self.tracking_descriptors = [descriptors[m.trainIdx] for m in good_matches]
        
    def estimate_pose(self, src_pts, dst_pts):
        """Estimate camera pose from matched points"""
        if self.camera_matrix is None:
            return
            
        # Essential matrix
        E, mask = cv2.findEssentialMat(src_pts, dst_pts, self.camera_matrix, 
                                      cv2.RANSAC, 0.999, 1.0)
        
        if E is None:
            return
            
        # Recover pose
        _, R, t, mask = cv2.recoverPose(E, src_pts, dst_pts, self.camera_matrix)
        
        # Update current pose
        if self.current_pose is None:
            self.current_pose = np.eye(4)
            
        # Update pose (simplified)
        pose_change = np.eye(4)
        pose_change[:3, :3] = R
        pose_change[:3, 3] = t.ravel()
        
        self.current_pose = self.current_pose @ pose_change
        
        # Publish pose
        self.publish_pose()
        
    def update_keyframes(self, keypoints, descriptors):
        """Update keyframe database"""
        if len(self.keyframes) == 0:
            # First keyframe
            self.keyframes.append({
                'keypoints': keypoints,
                'descriptors': descriptors,
                'pose': self.current_pose.copy() if self.current_pose is not None else np.eye(4)
            })
            return
            
        # Check if we need a new keyframe
        last_keyframe = self.keyframes[-1]
        
        # Simple keyframe selection based on distance
        if self.current_pose is not None:
            translation = np.linalg.norm(self.current_pose[:3, 3] - last_keyframe['pose'][:3, 3])
            
            if translation > self.min_distance:
                # Add new keyframe
                self.keyframes.append({
                    'keypoints': keypoints,
                    'descriptors': descriptors,
                    'pose': self.current_pose.copy()
                })
                
                # Check for loop closure
                self.check_loop_closure()
                
    def check_loop_closure(self):
        """Check for loop closure"""
        if len(self.keyframes) < 10:
            return
            
        current_kf = self.keyframes[-1]
        
        # Compare with previous keyframes
        for i, kf in enumerate(self.keyframes[:-5]):
            # Simple loop detection based on feature similarity
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(current_kf['descriptors'], kf['descriptors'])
            
            good_matches = [m for m in matches if m.distance < 30]
            
            if len(good_matches) > 20:
                self.loop_detected = True
                self.get_logger().info(f"Loop closure detected with keyframe {i}")
                break
                
    def publish_pose(self):
        """Publish current pose"""
        if self.current_pose is None:
            return
            
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Extract position and orientation
        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]
        
        # Convert rotation matrix to quaternion
        from scipy.spatial.transform import Rotation
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        self.pose_pub.publish(pose_msg)
        
        # Update path
        self.path.poses.append(pose_msg)
        self.path_pub.publish(self.path)
        
    def update_map(self):
        """Update occupancy grid map"""
        if self.current_pose is None:
            return
            
        # Simple map update based on current pose
        position = self.current_pose[:3, 3]
        
        # Convert to map coordinates
        map_x = int((position[0] - self.map_origin[0]) / self.map_resolution)
        map_y = int((position[1] - self.map_origin[1]) / self.map_resolution)
        
        if 0 <= map_x < self.map_pixels and 0 <= map_y < self.map_pixels:
            # Mark current position as free
            self.occupancy_map[map_y, map_x] = 0
            
            # Add some synthetic obstacles
            for i in range(5):
                obs_x = map_x + random.randint(-10, 10)
                obs_y = map_y + random.randint(-10, 10)
                if 0 <= obs_x < self.map_pixels and 0 <= obs_y < self.map_pixels:
                    self.occupancy_map[obs_y, obs_x] = 100
                    
    def publish_map(self):
        """Publish occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_pixels
        map_msg.info.height = self.map_pixels
        map_msg.info.origin.position.x = self.map_origin[0]
        map_msg.info.origin.position.y = self.map_origin[1]
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.occupancy_map.flatten().tolist()
        
        self.map_pub.publish(map_msg)
        
    def publish_markers(self):
        """Publish visualization markers"""
        markers = MarkerArray()
        
        # Keyframe markers
        for i, kf in enumerate(self.keyframes):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'keyframes'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            position = kf['pose'][:3, 3]
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            markers.markers.append(marker)
            
        self.markers_pub.publish(markers)
        
    def update_slam(self):
        """Main SLAM update loop"""
        with self.slam_lock:
            if self.is_initialized:
                self.update_map()
                self.publish_map()
                self.publish_markers()

def main(args=None):
    rclpy.init(args=args)
    visual_slam = VisualSLAM()
    
    try:
        rclpy.spin(visual_slam)
    except KeyboardInterrupt:
        visual_slam.get_logger().info("Visual SLAM stopped by user")
    finally:
        visual_slam.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 