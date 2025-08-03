#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from nav_msgs.msg import Odometry
import time
import math

class PhysicsTest(Node):
    def __init__(self):
        super().__init__('physics_test')
        
        # Test state
        self.imu_received = False
        self.gps_received = False
        self.altitude_received = False
        self.odom_received = False
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 'quadcopter/sensors/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, 'quadcopter/sensors/gps', self.gps_callback, 10)
        self.altitude_sub = self.create_subscription(
            LaserScan, 'quadcopter/sensors/altitude', self.altitude_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'quadcopter/state/odometry', self.odom_callback, 10)
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            Twist, 'quadcopter/control/velocity', 10)
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_tests)
        
        self.get_logger().info('Physics test initialized')
    
    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_received = True
        self.get_logger().info(f'IMU received - Angular velocity: {msg.angular_velocity}')
    
    def gps_callback(self, msg):
        """Handle GPS data"""
        self.gps_received = True
        self.get_logger().info(f'GPS received - Position: {msg.latitude}, {msg.longitude}, {msg.altitude}')
    
    def altitude_callback(self, msg):
        """Handle altitude data"""
        self.altitude_received = True
        if len(msg.ranges) > 0:
            self.get_logger().info(f'Altitude received: {msg.ranges[0]} m')
    
    def odom_callback(self, msg):
        """Handle odometry data"""
        self.odom_received = True
        pos = msg.pose.pose.position
        self.get_logger().info(f'Odometry received - Position: {pos.x}, {pos.y}, {pos.z}')
    
    def run_tests(self):
        """Run physics tests"""
        self.get_logger().info('Running physics tests...')
        
        # Test 1: Check if sensors are publishing
        if self.imu_received:
            self.get_logger().info('✓ IMU sensor working')
        else:
            self.get_logger().error('✗ IMU sensor not working')
        
        if self.gps_received:
            self.get_logger().info('✓ GPS sensor working')
        else:
            self.get_logger().error('✗ GPS sensor not working')
        
        if self.altitude_received:
            self.get_logger().info('✓ Altitude sensor working')
        else:
            self.get_logger().error('✗ Altitude sensor not working')
        
        if self.odom_received:
            self.get_logger().info('✓ Odometry working')
        else:
            self.get_logger().error('✗ Odometry not working')
        
        # Test 2: Send control command
        self.test_control()
    
    def test_control(self):
        """Test control commands"""
        cmd = Twist()
        cmd.linear.z = 0.5  # Small upward velocity
        self.velocity_pub.publish(cmd)
        self.get_logger().info('Published control command')

def main(args=None):
    rclpy.init(args=args)
    test_node = PhysicsTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 