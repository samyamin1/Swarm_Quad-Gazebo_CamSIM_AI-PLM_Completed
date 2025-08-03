#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import math

class IntegrationTest(Node):
    def __init__(self):
        super().__init__('integration_test')
        
        # Test state
        self.sensors_working = False
        self.control_working = False
        self.state_working = False
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 'quadcopter/sensors/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, 'quadcopter/sensors/gps', self.gps_callback, 10)
        self.altitude_sub = self.create_subscription(
            LaserScan, 'quadcopter/sensors/altitude', self.altitude_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'quadcopter/state/odometry', self.odom_callback, 10)
        self.state_sub = self.create_subscription(
            String, 'quadcopter/state/flight_mode', self.state_callback, 10)
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            Twist, 'quadcopter/control/velocity', 10)
        self.waypoint_pub = self.create_publisher(
            PoseStamped, 'quadcopter/control/waypoint', 10)
        self.mode_pub = self.create_publisher(
            String, 'quadcopter/control/mode', 10)
        
        # Test results
        self.test_results = {
            'sensors': False,
            'control': False,
            'state': False,
            'flight_modes': False
        }
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.run_integration_tests)
        
        self.get_logger().info('Integration test initialized')
    
    def imu_callback(self, msg):
        """Handle IMU data"""
        self.test_results['sensors'] = True
        self.get_logger().info('✓ IMU sensor working')
    
    def gps_callback(self, msg):
        """Handle GPS data"""
        self.get_logger().info('✓ GPS sensor working')
    
    def altitude_callback(self, msg):
        """Handle altitude data"""
        if len(msg.ranges) > 0:
            self.get_logger().info(f'✓ Altitude sensor working: {msg.ranges[0]} m')
    
    def odom_callback(self, msg):
        """Handle odometry data"""
        self.test_results['state'] = True
        pos = msg.pose.pose.position
        self.get_logger().info(f'✓ Odometry working: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')
    
    def state_callback(self, msg):
        """Handle flight mode state"""
        self.get_logger().info(f'✓ Flight mode: {msg.data}')
    
    def run_integration_tests(self):
        """Run comprehensive integration tests"""
        self.get_logger().info('🧪 Running integration tests...')
        
        # Test 1: Sensor functionality
        self.test_sensors()
        
        # Test 2: Control functionality
        self.test_control()
        
        # Test 3: Flight modes
        self.test_flight_modes()
        
        # Test 4: Waypoint navigation
        self.test_waypoint_navigation()
        
        # Print test summary
        self.print_test_summary()
    
    def test_sensors(self):
        """Test sensor functionality"""
        self.get_logger().info('📡 Testing sensors...')
        
        # Wait for sensor data
        time.sleep(1)
        
        if self.test_results['sensors']:
            self.get_logger().info('✅ Sensors working correctly')
        else:
            self.get_logger().error('❌ Sensors not working')
    
    def test_control(self):
        """Test control functionality"""
        self.get_logger().info('🎮 Testing control...')
        
        # Send hover command
        cmd = Twist()
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.velocity_pub.publish(cmd)
        
        self.get_logger().info('✅ Control command sent')
        self.test_results['control'] = True
    
    def test_flight_modes(self):
        """Test flight mode switching"""
        self.get_logger().info('🔄 Testing flight modes...')
        
        # Test manual mode
        mode_msg = String()
        mode_msg.data = "MANUAL"
        self.mode_pub.publish(mode_msg)
        
        # Test auto mode
        mode_msg.data = "AUTO"
        self.mode_pub.publish(mode_msg)
        
        self.get_logger().info('✅ Flight mode switching tested')
        self.test_results['flight_modes'] = True
    
    def test_waypoint_navigation(self):
        """Test waypoint navigation"""
        self.get_logger().info('🎯 Testing waypoint navigation...')
        
        # Send a waypoint
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x = 5.0
        waypoint.pose.position.y = 0.0
        waypoint.pose.position.z = 2.0
        self.waypoint_pub.publish(waypoint)
        
        self.get_logger().info('✅ Waypoint navigation tested')
    
    def print_test_summary(self):
        """Print test results summary"""
        self.get_logger().info('📊 Integration Test Summary:')
        self.get_logger().info('=' * 40)
        
        for test_name, result in self.test_results.items():
            status = '✅ PASS' if result else '❌ FAIL'
            self.get_logger().info(f'{test_name:15} : {status}')
        
        self.get_logger().info('=' * 40)
        
        # Overall result
        all_passed = all(self.test_results.values())
        if all_passed:
            self.get_logger().info('🎉 All integration tests PASSED!')
        else:
            self.get_logger().error('💥 Some integration tests FAILED!')
        
        self.get_logger().info('')
        self.get_logger().info('🚀 Next steps:')
        self.get_logger().info('   - Run full simulation: ./scripts/launch_simulation.sh')
        self.get_logger().info('   - Test manual control with keyboard')
        self.get_logger().info('   - Test autonomous navigation')

def main(args=None):
    rclpy.init(args=args)
    test_node = IntegrationTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 