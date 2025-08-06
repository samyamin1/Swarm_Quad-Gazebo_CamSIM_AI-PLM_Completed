#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import cv2
import time
import random
from threading import Lock

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        
        # Camera parameters
        self.camera_width = 1920
        self.camera_height = 1080
        self.fps = 30
        self.fov = 90  # degrees
        
        # Noise parameters
        self.noise_level = 0.02
        self.blur_level = 0.5
        self.exposure_variation = 0.1
        self.white_balance_shift = 0.05
        
        # Latency simulation
        self.processing_latency = 0.033  # 33ms
        self.transmission_latency = 0.010  # 10ms
        
        # Failure simulation
        self.failure_probability = 0.001  # 0.1% chance per frame
        self.failure_duration = 5.0  # seconds
        self.failure_start_time = None
        self.is_failed = False
        
        # Camera state
        self.camera_lock = Lock()
        self.last_frame_time = time.time()
        self.frame_count = 0
        
        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/quadcopter/sensors/camera/rgb', 10)
        self.thermal_pub = self.create_publisher(Image, '/quadcopter/sensors/camera/thermal', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, '/quadcopter/sensors/camera/compressed', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/quadcopter/sensors/camera/info', 10)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/quadcopter/pose', 
            self.pose_callback, 
            10
        )
        
        # Timer for camera updates
        self.camera_timer = self.create_timer(1.0/self.fps, self.update_camera)
        
        # Camera info
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'camera_link'
        self.camera_info.width = self.camera_width
        self.camera_info.height = self.camera_height
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        self.camera_info.k = [
            self.camera_width/2, 0.0, self.camera_width/2,
            0.0, self.camera_height/2, self.camera_height/2,
            0.0, 0.0, 1.0
        ]
        self.camera_info.p = [
            self.camera_width/2, 0.0, self.camera_width/2, 0.0,
            0.0, self.camera_height/2, self.camera_height/2, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        self.get_logger().info("Camera Simulator initialized")
        
    def pose_callback(self, msg):
        """Update camera pose for realistic simulation"""
        with self.camera_lock:
            # Store current pose for camera calculations
            self.current_pose = msg.pose
            
    def simulate_noise(self, image):
        """Add realistic camera noise"""
        # Gaussian noise
        noise = np.random.normal(0, self.noise_level * 255, image.shape).astype(np.uint8)
        noisy_image = cv2.add(image, noise)
        
        # Motion blur (simulated)
        if random.random() < 0.1:  # 10% chance of motion blur
            kernel_size = int(self.blur_level * 10) + 1
            kernel = np.ones((kernel_size, kernel_size), np.float32) / (kernel_size * kernel_size)
            noisy_image = cv2.filter2D(noisy_image, -1, kernel)
        
        # Exposure variation
        if random.random() < 0.05:  # 5% chance of exposure change
            exposure_factor = 1.0 + random.uniform(-self.exposure_variation, self.exposure_variation)
            noisy_image = cv2.convertScaleAbs(noisy_image, alpha=exposure_factor)
        
        # White balance shift
        if random.random() < 0.03:  # 3% chance of white balance shift
            b_shift = random.uniform(-self.white_balance_shift, self.white_balance_shift)
            g_shift = random.uniform(-self.white_balance_shift, self.white_balance_shift)
            r_shift = random.uniform(-self.white_balance_shift, self.white_balance_shift)
            
            noisy_image = cv2.convertScaleAbs(noisy_image, alpha=1.0, beta=0)
            noisy_image[:, :, 0] = cv2.add(noisy_image[:, :, 0], int(b_shift * 255))
            noisy_image[:, :, 1] = cv2.add(noisy_image[:, :, 1], int(g_shift * 255))
            noisy_image[:, :, 2] = cv2.add(noisy_image[:, :, 2], int(r_shift * 255))
        
        return noisy_image
        
    def simulate_thermal_image(self, rgb_image):
        """Convert RGB to thermal-like image"""
        # Convert to grayscale
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        
        # Apply thermal-like colormap
        thermal = cv2.applyColorMap(gray, cv2.COLORMAP_HOT)
        
        # Add thermal-specific noise
        thermal_noise = np.random.normal(0, 5, thermal.shape).astype(np.uint8)
        thermal = cv2.add(thermal, thermal_noise)
        
        # Simulate thermal blooming
        if random.random() < 0.02:  # 2% chance of thermal blooming
            kernel = np.ones((5, 5), np.float32) / 25
            thermal = cv2.filter2D(thermal, -1, kernel)
        
        return thermal
        
    def check_failure(self):
        """Simulate camera failures"""
        current_time = time.time()
        
        # Check if we should start a failure
        if not self.is_failed and random.random() < self.failure_probability:
            self.is_failed = True
            self.failure_start_time = current_time
            self.get_logger().warn("🚨 Camera failure detected!")
            
        # Check if failure should end
        if self.is_failed and current_time - self.failure_start_time > self.failure_duration:
            self.is_failed = False
            self.get_logger().info("✅ Camera recovered from failure")
            
        return self.is_failed
        
    def generate_synthetic_image(self):
        """Generate synthetic camera image based on environment"""
        # Create a synthetic image based on quadcopter position
        image = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        
        # Add some synthetic environment features
        # Sky gradient
        for y in range(self.camera_height):
            intensity = int(255 * (1 - y / self.camera_height))
            image[y, :] = [intensity, intensity, 255]
        
        # Add some buildings/objects
        for i in range(5):
            x = random.randint(0, self.camera_width)
            y = random.randint(self.camera_height//2, self.camera_height)
            w = random.randint(50, 200)
            h = random.randint(100, 300)
            color = [random.randint(100, 200)] * 3
            cv2.rectangle(image, (x, y), (x+w, y-h), color, -1)
        
        # Add some moving objects (simulated)
        if random.random() < 0.3:
            x = random.randint(0, self.camera_width)
            y = random.randint(0, self.camera_height//2)
            cv2.circle(image, (x, y), 20, [255, 0, 0], -1)
        
        return image
        
    def update_camera(self):
        """Main camera update loop"""
        with self.camera_lock:
            current_time = time.time()
            
            # Check for failures
            if self.check_failure():
                # During failure, send corrupted or no data
                if random.random() < 0.5:
                    # Send corrupted image
                    corrupted_image = np.random.randint(0, 255, (self.camera_height, self.camera_width, 3), dtype=np.uint8)
                    self.publish_image(corrupted_image, 'rgb', current_time)
                return
            
            # Generate synthetic image
            image = self.generate_synthetic_image()
            
            # Add realistic noise
            noisy_image = self.simulate_noise(image)
            
            # Simulate processing latency
            time.sleep(self.processing_latency)
            
            # Publish RGB image
            self.publish_image(noisy_image, 'rgb', current_time)
            
            # Generate and publish thermal image
            thermal_image = self.simulate_thermal_image(noisy_image)
            self.publish_image(thermal_image, 'thermal', current_time)
            
            # Publish camera info
            self.camera_info.header.stamp = self.get_clock().now().to_msg()
            self.camera_info_pub.publish(self.camera_info)
            
            self.frame_count += 1
            
    def publish_image(self, image, image_type, timestamp):
        """Publish image with proper ROS message formatting"""
        # Convert to ROS Image message
        ros_image = Image()
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = f'camera_{image_type}_link'
        ros_image.height = image.shape[0]
        ros_image.width = image.shape[1]
        ros_image.encoding = 'bgr8'
        ros_image.step = image.shape[1] * 3
        ros_image.data = image.tobytes()
        
        # Add transmission latency
        time.sleep(self.transmission_latency)
        
        # Publish based on type
        if image_type == 'rgb':
            self.rgb_pub.publish(ros_image)
        elif image_type == 'thermal':
            self.thermal_pub.publish(ros_image)
            
        # Also publish compressed version
        compressed_msg = CompressedImage()
        compressed_msg.header = ros_image.header
        compressed_msg.format = 'jpeg'
        compressed_msg.data = cv2.imencode('.jpg', image)[1].tobytes()
        self.compressed_pub.publish(compressed_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_sim = CameraSimulator()
    
    try:
        rclpy.spin(camera_sim)
    except KeyboardInterrupt:
        camera_sim.get_logger().info("Camera simulator stopped by user")
    finally:
        camera_sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 