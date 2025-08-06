#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import struct
import time
import random
from threading import Lock

class LidarSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')
        
        # Lidar parameters
        self.scan_range = 100.0  # meters
        self.scan_angle = 360.0  # degrees
        self.scan_resolution = 0.1  # degrees
        self.scan_frequency = 10  # Hz
        self.num_points = int(self.scan_angle / self.scan_resolution)
        
        # Noise parameters
        self.range_noise = 0.1  # meters
        self.angle_noise = 0.01  # degrees
        self.intensity_noise = 0.05
        
        # Environmental effects
        self.rain_effect = 0.0  # 0-1, affects range
        self.fog_effect = 0.0  # 0-1, affects intensity
        self.dust_effect = 0.0  # 0-1, affects point density
        
        # Failure simulation
        self.failure_probability = 0.0005  # 0.05% chance per scan
        self.failure_duration = 3.0  # seconds
        self.failure_start_time = None
        self.is_failed = False
        
        # Lidar state
        self.lidar_lock = Lock()
        self.current_pose = None
        self.scan_count = 0
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/quadcopter/sensors/lidar/pointcloud', 10)
        self.laserscan_pub = self.create_publisher(LaserScan, '/quadcopter/sensors/lidar/scan', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/quadcopter/pose', 
            self.pose_callback, 
            10
        )
        
        # Timer for lidar updates
        self.lidar_timer = self.create_timer(1.0/self.scan_frequency, self.update_lidar)
        
        self.get_logger().info("Lidar Simulator initialized")
        
    def pose_callback(self, msg):
        """Update lidar pose for realistic simulation"""
        with self.lidar_lock:
            self.current_pose = msg.pose
            
    def simulate_environment(self):
        """Simulate environmental effects on lidar"""
        # Simulate rain (reduces range)
        if random.random() < 0.1:  # 10% chance of rain
            self.rain_effect = random.uniform(0.1, 0.3)
        else:
            self.rain_effect = max(0, self.rain_effect - 0.01)
            
        # Simulate fog (reduces intensity)
        if random.random() < 0.05:  # 5% chance of fog
            self.fog_effect = random.uniform(0.2, 0.5)
        else:
            self.fog_effect = max(0, self.fog_effect - 0.005)
            
        # Simulate dust (reduces point density)
        if random.random() < 0.03:  # 3% chance of dust
            self.dust_effect = random.uniform(0.1, 0.4)
        else:
            self.dust_effect = max(0, self.dust_effect - 0.01)
            
    def generate_synthetic_scan(self):
        """Generate synthetic lidar scan data"""
        angles = np.linspace(0, np.radians(self.scan_angle), self.num_points)
        ranges = np.full(self.num_points, self.scan_range)
        intensities = np.full(self.num_points, 1.0)
        
        # Add synthetic obstacles based on environment
        for i in range(10):  # 10 random obstacles
            obstacle_angle = random.uniform(0, 2*np.pi)
            obstacle_range = random.uniform(5, 50)
            obstacle_width = random.uniform(0.1, 0.5)
            
            # Find closest angle index
            angle_idx = np.argmin(np.abs(angles - obstacle_angle))
            
            # Create obstacle in scan
            obstacle_points = int(obstacle_width / np.radians(self.scan_resolution))
            start_idx = max(0, angle_idx - obstacle_points//2)
            end_idx = min(self.num_points, angle_idx + obstacle_points//2)
            
            for j in range(start_idx, end_idx):
                ranges[j] = obstacle_range
                intensities[j] = random.uniform(0.7, 1.0)
        
        # Add ground plane reflection
        ground_angle_start = int(self.num_points * 0.4)  # 40% of scan
        ground_angle_end = int(self.num_points * 0.6)    # 60% of scan
        
        for i in range(ground_angle_start, ground_angle_end):
            ranges[i] = min(ranges[i], random.uniform(2, 10))
            intensities[i] = random.uniform(0.3, 0.6)
        
        return angles, ranges, intensities
        
    def add_noise(self, angles, ranges, intensities):
        """Add realistic lidar noise"""
        # Range noise
        range_noise = np.random.normal(0, self.range_noise, ranges.shape)
        ranges += range_noise
        ranges = np.clip(ranges, 0, self.scan_range)
        
        # Angle noise
        angle_noise = np.random.normal(0, np.radians(self.angle_noise), angles.shape)
        angles += angle_noise
        
        # Intensity noise
        intensity_noise = np.random.normal(0, self.intensity_noise, intensities.shape)
        intensities += intensity_noise
        intensities = np.clip(intensities, 0, 1)
        
        # Environmental effects
        ranges *= (1 - self.rain_effect)
        intensities *= (1 - self.fog_effect)
        
        return angles, ranges, intensities
        
    def check_failure(self):
        """Simulate lidar failures"""
        current_time = time.time()
        
        # Check if we should start a failure
        if not self.is_failed and random.random() < self.failure_probability:
            self.is_failed = True
            self.failure_start_time = current_time
            self.get_logger().warn("🚨 Lidar failure detected!")
            
        # Check if failure should end
        if self.is_failed and current_time - self.failure_start_time > self.failure_duration:
            self.is_failed = False
            self.get_logger().info("✅ Lidar recovered from failure")
            
        return self.is_failed
        
    def create_pointcloud(self, angles, ranges, intensities):
        """Convert scan data to PointCloud2 message"""
        # Filter valid points
        valid_mask = (ranges > 0) & (ranges < self.scan_range)
        valid_angles = angles[valid_mask]
        valid_ranges = ranges[valid_mask]
        valid_intensities = intensities[valid_mask]
        
        # Convert to Cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        z = np.zeros_like(x)  # 2D lidar
        
        # Create point cloud data
        points = []
        for i in range(len(x)):
            # Pack point data (x, y, z, intensity)
            point_data = struct.pack('ffff', x[i], y[i], z[i], valid_intensities[i])
            points.append(point_data)
        
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'lidar_link'
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        
        # Set point field information
        cloud_msg.fields = [
            {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},
            {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
            {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1},
            {'name': 'intensity', 'offset': 12, 'datatype': 7, 'count': 1}
        ]
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = b''.join(points)
        cloud_msg.is_dense = True
        
        return cloud_msg
        
    def create_laserscan(self, angles, ranges, intensities):
        """Convert scan data to LaserScan message"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'lidar_link'
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = np.radians(self.scan_angle)
        scan_msg.angle_increment = np.radians(self.scan_resolution)
        scan_msg.time_increment = 1.0 / (self.scan_frequency * self.num_points)
        scan_msg.scan_time = 1.0 / self.scan_frequency
        scan_msg.range_min = 0.1
        scan_msg.range_max = self.scan_range
        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = intensities.tolist()
        
        return scan_msg
        
    def update_lidar(self):
        """Main lidar update loop"""
        with self.lidar_lock:
            current_time = time.time()
            
            # Simulate environmental effects
            self.simulate_environment()
            
            # Check for failures
            if self.check_failure():
                # During failure, send corrupted or no data
                if random.random() < 0.3:
                    # Send corrupted scan
                    corrupted_ranges = np.random.uniform(0, self.scan_range, self.num_points)
                    corrupted_intensities = np.random.uniform(0, 1, self.num_points)
                    angles = np.linspace(0, np.radians(self.scan_angle), self.num_points)
                    
                    scan_msg = self.create_laserscan(angles, corrupted_ranges, corrupted_intensities)
                    self.laserscan_pub.publish(scan_msg)
                return
            
            # Generate synthetic scan
            angles, ranges, intensities = self.generate_synthetic_scan()
            
            # Add realistic noise
            angles, ranges, intensities = self.add_noise(angles, ranges, intensities)
            
            # Apply dust effect (reduce point density)
            if self.dust_effect > 0:
                dust_mask = np.random.random(self.num_points) > self.dust_effect
                ranges[~dust_mask] = self.scan_range  # Mark as no return
                intensities[~dust_mask] = 0
            
            # Create and publish messages
            cloud_msg = self.create_pointcloud(angles, ranges, intensities)
            scan_msg = self.create_laserscan(angles, ranges, intensities)
            
            self.pointcloud_pub.publish(cloud_msg)
            self.laserscan_pub.publish(scan_msg)
            
            self.scan_count += 1
            
            # Log statistics periodically
            if self.scan_count % 100 == 0:
                valid_points = np.sum((ranges > 0) & (ranges < self.scan_range))
                self.get_logger().info(f"Lidar scan {self.scan_count}: {valid_points} valid points, "
                                     f"rain: {self.rain_effect:.2f}, fog: {self.fog_effect:.2f}, "
                                     f"dust: {self.dust_effect:.2f}")

def main(args=None):
    rclpy.init(args=args)
    lidar_sim = LidarSimulator()
    
    try:
        rclpy.spin(lidar_sim)
    except KeyboardInterrupt:
        lidar_sim.get_logger().info("Lidar simulator stopped by user")
    finally:
        lidar_sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 