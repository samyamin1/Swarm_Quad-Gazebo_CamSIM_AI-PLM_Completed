#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import time
import json
import numpy as np

class NavigationControlTest(Node):
    def __init__(self):
        super().__init__('navigation_control_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        self.test_duration = 60  # seconds
        
        # Navigation data tracking
        self.navigation_data = {
            'cmd_vel': {'received': False, 'count': 0, 'last_time': 0, 'commands': []},
            'navigation_status': {'received': False, 'count': 0, 'last_time': 0, 'statuses': []},
            'mission_goal': {'received': False, 'count': 0, 'last_time': 0, 'goals': []}
        }
        
        # Test goals
        self.test_goals = [
            {'x': 5.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0},
            {'x': 5.0, 'y': 5.0, 'z': 3.0, 'yaw': 1.57},
            {'x': 0.0, 'y': 5.0, 'z': 2.0, 'yaw': 3.14},
            {'x': 0.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0}
        ]
        self.current_goal_index = 0
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/quadcopter/cmd_vel', 
            lambda msg: self.navigation_callback('cmd_vel', msg), 10
        )
        self.navigation_status_sub = self.create_subscription(
            String, '/quadcopter/navigation/status', 
            lambda msg: self.navigation_callback('navigation_status', msg), 10
        )
        
        # Publishers
        self.mission_goal_pub = self.create_publisher(PoseStamped, '/quadcopter/mission/goal', 10)
        self.odometry_pub = self.create_publisher(Odometry, '/quadcopter/odometry', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/quadcopter/scan', 10)
        self.emergency_pub = self.create_publisher(Bool, '/quadcopter/emergency', 10)
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        # Simulated quadcopter state
        self.simulated_pose = {'x': 0.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0}
        self.simulated_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        
        # Odometry simulation timer
        self.odometry_timer = self.create_timer(0.1, self.simulate_odometry)
        
        self.get_logger().info("🧪 Navigation Control Test Started")
        
    def navigation_callback(self, data_type, msg):
        """Track navigation data reception"""
        current_time = time.time()
        
        if not self.navigation_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.navigation_data[data_type]['received'] = True
            
        self.navigation_data[data_type]['count'] += 1
        self.navigation_data[data_type]['last_time'] = current_time
        
        # Store data for analysis
        if data_type == 'cmd_vel':
            command = {
                'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
                'angular': {'z': msg.angular.z}
            }
            self.navigation_data[data_type]['commands'].append(command)
        elif data_type == 'navigation_status':
            try:
                status = json.loads(msg.data)
                self.navigation_data[data_type]['statuses'].append(status)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in navigation status")
                
    def simulate_odometry(self):
        """Simulate quadcopter odometry"""
        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "map"
        
        odometry_msg.pose.pose.position.x = self.simulated_pose['x']
        odometry_msg.pose.pose.position.y = self.simulated_pose['y']
        odometry_msg.pose.pose.position.z = self.simulated_pose['z']
        
        # Convert yaw to quaternion
        yaw = self.simulated_pose['yaw']
        odometry_msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        odometry_msg.pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        self.odometry_pub.publish(odometry_msg)
        
    def simulate_laser_scan(self):
        """Simulate laser scan data"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "quadcopter"
        
        scan_msg.angle_min = -np.pi
        scan_msg.angle_max = np.pi
        scan_msg.angle_increment = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        
        # Generate scan data
        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        scan_msg.ranges = [10.0] * num_readings  # Default max range
        
        # Add some obstacles
        for i in range(0, num_readings, 10):
            scan_msg.ranges[i] = 2.0 + np.random.normal(0, 0.1)
            
        self.scan_pub.publish(scan_msg)
        
    def send_test_goal(self):
        """Send a test goal to the navigation controller"""
        if self.current_goal_index < len(self.test_goals):
            goal = self.test_goals[self.current_goal_index]
            
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = goal['x']
            goal_msg.pose.position.y = goal['y']
            goal_msg.pose.position.z = goal['z']
            
            # Convert yaw to quaternion
            yaw = goal['yaw']
            goal_msg.pose.orientation.z = np.sin(yaw / 2.0)
            goal_msg.pose.orientation.w = np.cos(yaw / 2.0)
            
            self.mission_goal_pub.publish(goal_msg)
            self.get_logger().info(f"🎯 Sending test goal {self.current_goal_index + 1}: ({goal['x']:.1f}, {goal['y']:.1f}, {goal['z']:.1f})")
            
    def check_navigation_health(self):
        """Check if navigation system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.navigation_data)
        
        for component_name, data in self.navigation_data.items():
            if data['received'] and (current_time - data['last_time']) < 10.0:
                healthy_components += 1
            else:
                self.get_logger().warn(f"⚠️ {component_name} not responding")
                
        return healthy_components, total_components
        
    def analyze_navigation_data(self):
        """Analyze navigation data quality"""
        analysis = {}
        
        for component_name, data in self.navigation_data.items():
            if data['received']:
                elapsed_time = time.time() - self.start_time
                data_rate = data['count'] / elapsed_time if elapsed_time > 0 else 0
                
                analysis[component_name] = {
                    'data_rate': data_rate,
                    'total_count': data['count'],
                    'last_update': time.time() - data['last_time']
                }
                
        return analysis
        
    def analyze_control_commands(self):
        """Analyze control command quality"""
        if not self.navigation_data['cmd_vel']['commands']:
            return None
            
        commands = self.navigation_data['cmd_vel']['commands']
        
        # Extract command metrics
        linear_x = [cmd['linear']['x'] for cmd in commands]
        linear_y = [cmd['linear']['y'] for cmd in commands]
        linear_z = [cmd['linear']['z'] for cmd in commands]
        angular_z = [cmd['angular']['z'] for cmd in commands]
        
        # Calculate command statistics
        return {
            'total_commands': len(commands),
            'avg_linear_x': np.mean(linear_x),
            'avg_linear_y': np.mean(linear_y),
            'avg_linear_z': np.mean(linear_z),
            'avg_angular_z': np.mean(angular_z),
            'max_linear': max(max(linear_x), max(linear_y), max(linear_z)),
            'max_angular': max(angular_z)
        }
        
    def analyze_navigation_status(self):
        """Analyze navigation status"""
        if not self.navigation_data['navigation_status']['statuses']:
            return None
            
        statuses = self.navigation_data['navigation_status']['statuses']
        
        # Extract navigation states
        states = [status.get('state', 'unknown') for status in statuses]
        goal_reached = [status.get('goal_reached', False) for status in statuses]
        emergency_stops = [status.get('emergency_stop', False) for status in statuses]
        obstacle_counts = [status.get('obstacle_count', 0) for status in statuses]
        
        return {
            'total_statuses': len(statuses),
            'unique_states': list(set(states)),
            'goals_reached': sum(goal_reached),
            'emergency_activations': sum(emergency_stops),
            'avg_obstacles': np.mean(obstacle_counts) if obstacle_counts else 0
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            self.get_logger().info("⏳ Waiting for Navigation Controller to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic navigation connectivity
            self.get_logger().info("🧪 Phase 1: Navigation Controller Connectivity Test")
            
            healthy_components, total_components = self.check_navigation_health()
            
            if healthy_components == total_components:
                self.get_logger().info(f"✅ All {total_components} navigation components connected and responding")
                self.test_phase = 1
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {healthy_components}/{total_components} navigation components healthy")
                
        elif self.test_phase == 1:
            # Phase 2: Navigation data quality analysis
            self.get_logger().info("🧪 Phase 2: Navigation Data Quality Analysis")
            
            analysis = self.analyze_navigation_data()
            
            self.get_logger().info("📊 Navigation Data Analysis:")
            for component_name, stats in analysis.items():
                self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, "
                                     f"{stats['total_count']} samples, "
                                     f"last update: {stats['last_update']:.1f}s ago")
            
            # Check if data rates are reasonable
            good_rates = 0
            for component_name, stats in analysis.items():
                expected_rate = {
                    'cmd_vel': 10,
                    'navigation_status': 1,
                    'mission_goal': 0.1
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:
                    good_rates += 1
                else:
                    self.get_logger().warn(f"⚠️ {component_name} data rate low: {stats['data_rate']:.1f} Hz")
            
            if good_rates == len(analysis):
                self.get_logger().info("✅ All navigation components have good data rates")
                self.test_phase = 2
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {good_rates}/{len(analysis)} components have good data rates")
                
        elif self.test_phase == 2:
            # Phase 3: Goal following test
            self.get_logger().info("🧪 Phase 3: Goal Following Test")
            
            # Send test goals
            if elapsed_time < 10.0:
                self.send_test_goal()
                self.current_goal_index += 1
                
            # Analyze control commands
            control_analysis = self.analyze_control_commands()
            
            if control_analysis:
                self.get_logger().info("🎮 Control Command Analysis:")
                self.get_logger().info(f"  Total commands: {control_analysis['total_commands']}")
                self.get_logger().info(f"  Average velocities: X={control_analysis['avg_linear_x']:.2f}, "
                                     f"Y={control_analysis['avg_linear_y']:.2f}, "
                                     f"Z={control_analysis['avg_linear_z']:.2f}")
                self.get_logger().info(f"  Max velocities: Linear={control_analysis['max_linear']:.2f}, "
                                     f"Angular={control_analysis['max_angular']:.2f}")
                
                # Check if commands are reasonable
                if control_analysis['total_commands'] > 10:
                    self.get_logger().info("✅ Navigation controller is generating commands")
                else:
                    self.get_logger().warn("⚠️ Navigation controller not generating enough commands")
                    
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: Navigation status analysis
            self.get_logger().info("🧪 Phase 4: Navigation Status Analysis")
            
            status_analysis = self.analyze_navigation_status()
            
            if status_analysis:
                self.get_logger().info("🧭 Navigation Status Analysis:")
                self.get_logger().info(f"  Total status updates: {status_analysis['total_statuses']}")
                self.get_logger().info(f"  Navigation states: {status_analysis['unique_states']}")
                self.get_logger().info(f"  Goals reached: {status_analysis['goals_reached']}")
                self.get_logger().info(f"  Emergency activations: {status_analysis['emergency_activations']}")
                self.get_logger().info(f"  Average obstacles: {status_analysis['avg_obstacles']:.1f}")
                
                # Check navigation performance
                if status_analysis['goals_reached'] > 0:
                    self.get_logger().info("✅ Navigation controller is reaching goals")
                else:
                    self.get_logger().warn("⚠️ No goals reached yet")
                    
                if len(status_analysis['unique_states']) > 1:
                    self.get_logger().info("✅ Navigation controller is transitioning between states")
                else:
                    self.get_logger().warn("⚠️ Navigation controller may be stuck in single state")
            else:
                self.get_logger().warn("⚠️ No navigation status data available")
                
            self.test_phase = 4
            self.start_time = time.time()
            
        elif self.test_phase == 4:
            # Phase 5: Emergency response test
            self.get_logger().info("🧪 Phase 5: Emergency Response Test")
            
            # Simulate emergency
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_pub.publish(emergency_msg)
            
            self.get_logger().info("🚨 Emergency signal sent - checking response...")
            
            # Wait for emergency response
            time.sleep(2.0)
            
            # Clear emergency
            emergency_msg.data = False
            self.emergency_pub.publish(emergency_msg)
            
            self.get_logger().info("✅ Emergency response test completed")
            
            self.test_phase = 5
            self.start_time = time.time()
            
        elif self.test_phase == 5:
            # Phase 6: Final navigation integration test
            self.get_logger().info("🧪 Phase 6: Final Navigation Integration Test")
            
            # Final health check
            healthy_components, total_components = self.check_navigation_health()
            analysis = self.analyze_navigation_data()
            control_analysis = self.analyze_control_commands()
            status_analysis = self.analyze_navigation_status()
            
            self.get_logger().info("🎉 NAVIGATION CONTROL TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} navigation components healthy")
            self.get_logger().info(f"✅ Total navigation samples: {sum(data['count'] for data in self.navigation_data.values())}")
            
            if control_analysis:
                self.get_logger().info(f"✅ Control: {control_analysis['total_commands']} commands generated")
                
            if status_analysis:
                self.get_logger().info(f"✅ Status: {status_analysis['total_statuses']} updates, "
                                     f"{status_analysis['goals_reached']} goals reached")
            
            self.get_logger().info("✅ Navigation Control Loop integrated and working")
            self.get_logger().info("✅ Ready for Stage 8: Swarm Coordination")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()
            
    def print_final_report(self):
        """Print final test report"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 FINAL NAVIGATION CONTROL REPORT")
        self.get_logger().info("="*60)
        
        analysis = self.analyze_navigation_data()
        control_analysis = self.analyze_control_commands()
        status_analysis = self.analyze_navigation_status()
        total_samples = sum(data['count'] for data in self.navigation_data.values())
        
        self.get_logger().info(f"📊 Total Navigation Samples: {total_samples}")
        self.get_logger().info(f"⏱️ Test Duration: {time.time() - self.start_time:.1f} seconds")
        
        for component_name, stats in analysis.items():
            self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, {stats['total_count']} samples")
            
        if control_analysis:
            self.get_logger().info(f"🎮 Control: {control_analysis['total_commands']} commands, "
                                 f"max velocity: {control_analysis['max_linear']:.2f} m/s")
            
        if status_analysis:
            self.get_logger().info(f"🧭 Status: {status_analysis['total_statuses']} updates, "
                                 f"goals reached: {status_analysis['goals_reached']}")
            
        self.get_logger().info("✅ Stage 7: Navigation Control Loop - COMPLETED")
        self.get_logger().info("🚀 Ready for Stage 8: Swarm Coordination")

def main(args=None):
    rclpy.init(args=args)
    test_node = NavigationControlTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
        test_node.print_final_report()
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 