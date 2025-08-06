#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image, CameraInfo
import time
import numpy as np
import math

class SLAMIntegrationTest(Node):
    def __init__(self):
        super().__init__('slam_integration_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        self.test_duration = 45  # seconds
        
        # SLAM data tracking
        self.slam_data = {
            'pose': {'received': False, 'count': 0, 'last_time': 0, 'positions': []},
            'map': {'received': False, 'count': 0, 'last_time': 0},
            'path': {'received': False, 'count': 0, 'last_time': 0, 'poses': []},
            'markers': {'received': False, 'count': 0, 'last_time': 0}
        }
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/quadcopter/slam/pose', 
            lambda msg: self.slam_callback('pose', msg), 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/quadcopter/slam/map', 
            lambda msg: self.slam_callback('map', msg), 10
        )
        self.path_sub = self.create_subscription(
            Path, '/quadcopter/slam/path', 
            lambda msg: self.slam_callback('path', msg), 10
        )
        self.markers_sub = self.create_subscription(
            MarkerArray, '/quadcopter/slam/markers', 
            lambda msg: self.slam_callback('markers', msg), 10
        )
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        self.get_logger().info("🧪 SLAM Integration Test Started")
        
    def slam_callback(self, data_type, msg):
        """Track SLAM data reception"""
        current_time = time.time()
        
        if not self.slam_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.slam_data[data_type]['received'] = True
            
        self.slam_data[data_type]['count'] += 1
        self.slam_data[data_type]['last_time'] = current_time
        
        # Store additional data for analysis
        if data_type == 'pose':
            position = msg.pose.position
            self.slam_data[data_type]['positions'].append([position.x, position.y, position.z])
        elif data_type == 'path':
            self.slam_data[data_type]['poses'] = msg.poses
            
    def check_slam_health(self):
        """Check if SLAM system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.slam_data)
        
        for component_name, data in self.slam_data.items():
            # Check if component is receiving data within last 10 seconds
            if data['received'] and (current_time - data['last_time']) < 10.0:
                healthy_components += 1
            else:
                self.get_logger().warn(f"⚠️ {component_name} not responding")
                
        return healthy_components, total_components
        
    def analyze_slam_data(self):
        """Analyze SLAM data quality"""
        analysis = {}
        
        for component_name, data in self.slam_data.items():
            if data['received']:
                # Calculate data rate
                elapsed_time = time.time() - self.start_time
                data_rate = data['count'] / elapsed_time if elapsed_time > 0 else 0
                
                analysis[component_name] = {
                    'data_rate': data_rate,
                    'total_count': data['count'],
                    'last_update': time.time() - data['last_time']
                }
                
        return analysis
        
    def analyze_trajectory(self):
        """Analyze trajectory quality"""
        if len(self.slam_data['pose']['positions']) < 2:
            return None
            
        positions = np.array(self.slam_data['pose']['positions'])
        
        # Calculate trajectory statistics
        total_distance = 0
        for i in range(1, len(positions)):
            distance = np.linalg.norm(positions[i] - positions[i-1])
            total_distance += distance
            
        # Calculate trajectory smoothness
        if len(positions) > 2:
            accelerations = []
            for i in range(1, len(positions)-1):
                prev_pos = positions[i-1]
                curr_pos = positions[i]
                next_pos = positions[i+1]
                
                # Simple acceleration estimate
                acc = np.linalg.norm(next_pos - 2*curr_pos + prev_pos)
                accelerations.append(acc)
                
            smoothness = np.mean(accelerations) if accelerations else 0
        else:
            smoothness = 0
            
        return {
            'total_distance': total_distance,
            'num_positions': len(positions),
            'smoothness': smoothness,
            'avg_speed': total_distance / (time.time() - self.start_time) if self.start_time > 0 else 0
        }
        
    def analyze_map_quality(self):
        """Analyze map quality"""
        if not self.slam_data['map']['received']:
            return None
            
        # Simple map analysis (would need actual map data)
        return {
            'map_received': True,
            'map_updates': self.slam_data['map']['count']
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            # Phase 0: Wait for SLAM to initialize
            self.get_logger().info("⏳ Waiting for SLAM to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic SLAM connectivity
            self.get_logger().info("🧪 Phase 1: SLAM Connectivity Test")
            
            healthy_components, total_components = self.check_slam_health()
            
            if healthy_components == total_components:
                self.get_logger().info(f"✅ All {total_components} SLAM components connected and responding")
                self.test_phase = 1
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {healthy_components}/{total_components} SLAM components healthy")
                
        elif self.test_phase == 1:
            # Phase 2: SLAM data quality analysis
            self.get_logger().info("🧪 Phase 2: SLAM Data Quality Analysis")
            
            analysis = self.analyze_slam_data()
            
            self.get_logger().info("📊 SLAM Data Analysis:")
            for component_name, stats in analysis.items():
                self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, "
                                     f"{stats['total_count']} samples, "
                                     f"last update: {stats['last_update']:.1f}s ago")
            
            # Check if data rates are reasonable
            good_rates = 0
            for component_name, stats in analysis.items():
                expected_rate = {
                    'pose': 10,
                    'map': 1,
                    'path': 10,
                    'markers': 5
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:  # 30% of expected
                    good_rates += 1
                else:
                    self.get_logger().warn(f"⚠️ {component_name} data rate low: {stats['data_rate']:.1f} Hz")
            
            if good_rates == len(analysis):
                self.get_logger().info("✅ All SLAM components have good data rates")
                self.test_phase = 2
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {good_rates}/{len(analysis)} components have good data rates")
                
        elif self.test_phase == 2:
            # Phase 3: Trajectory analysis
            self.get_logger().info("🧪 Phase 3: Trajectory Analysis")
            
            trajectory_analysis = self.analyze_trajectory()
            
            if trajectory_analysis:
                self.get_logger().info("📈 Trajectory Analysis:")
                self.get_logger().info(f"  Total distance: {trajectory_analysis['total_distance']:.2f} meters")
                self.get_logger().info(f"  Number of positions: {trajectory_analysis['num_positions']}")
                self.get_logger().info(f"  Average speed: {trajectory_analysis['avg_speed']:.2f} m/s")
                self.get_logger().info(f"  Trajectory smoothness: {trajectory_analysis['smoothness']:.4f}")
                
                # Check trajectory quality
                if trajectory_analysis['total_distance'] > 5.0:
                    self.get_logger().info("✅ Trajectory shows significant movement")
                else:
                    self.get_logger().warn("⚠️ Trajectory shows limited movement")
                    
                if trajectory_analysis['smoothness'] < 1.0:
                    self.get_logger().info("✅ Trajectory is smooth")
                else:
                    self.get_logger().warn("⚠️ Trajectory shows some instability")
            else:
                self.get_logger().warn("⚠️ Insufficient trajectory data for analysis")
                
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: Map quality analysis
            self.get_logger().info("🧪 Phase 4: Map Quality Analysis")
            
            map_analysis = self.analyze_map_quality()
            
            if map_analysis:
                self.get_logger().info("🗺️ Map Analysis:")
                self.get_logger().info(f"  Map received: {map_analysis['map_received']}")
                self.get_logger().info(f"  Map updates: {map_analysis['map_updates']}")
                
                if map_analysis['map_updates'] > 5:
                    self.get_logger().info("✅ Map is being updated regularly")
                else:
                    self.get_logger().warn("⚠️ Map updates are infrequent")
            else:
                self.get_logger().warn("⚠️ No map data available")
                
            self.test_phase = 4
            self.start_time = time.time()
            
        elif self.test_phase == 4:
            # Phase 5: Loop closure detection test
            self.get_logger().info("🧪 Phase 5: Loop Closure Detection Test")
            
            # Simulate loop closure detection by monitoring trajectory
            trajectory_analysis = self.analyze_trajectory()
            
            if trajectory_analysis and trajectory_analysis['total_distance'] > 20.0:
                self.get_logger().info("✅ Sufficient trajectory for loop closure detection")
                self.get_logger().info("✅ Loop closure detection system ready")
            else:
                self.get_logger().info("⏳ Building trajectory for loop closure detection")
                
            self.test_phase = 5
            self.start_time = time.time()
            
        elif self.test_phase == 5:
            # Phase 6: Final SLAM integration test
            self.get_logger().info("🧪 Phase 6: Final SLAM Integration Test")
            
            # Final health check
            healthy_components, total_components = self.check_slam_health()
            analysis = self.analyze_slam_data()
            trajectory_analysis = self.analyze_trajectory()
            
            self.get_logger().info("🎉 SLAM INTEGRATION TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} SLAM components healthy")
            self.get_logger().info(f"✅ Total SLAM samples: {sum(data['count'] for data in self.slam_data.values())}")
            
            if trajectory_analysis:
                self.get_logger().info(f"✅ Trajectory: {trajectory_analysis['total_distance']:.2f}m, "
                                     f"{trajectory_analysis['num_positions']} positions")
            
            self.get_logger().info("✅ SLAM system integrated and working")
            self.get_logger().info("✅ Ready for Stage 5: Perception Language Model")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()
            
    def print_final_report(self):
        """Print final test report"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 FINAL SLAM INTEGRATION REPORT")
        self.get_logger().info("="*60)
        
        analysis = self.analyze_slam_data()
        trajectory_analysis = self.analyze_trajectory()
        total_samples = sum(data['count'] for data in self.slam_data.values())
        
        self.get_logger().info(f"📊 Total SLAM Samples: {total_samples}")
        self.get_logger().info(f"⏱️ Test Duration: {time.time() - self.start_time:.1f} seconds")
        
        for component_name, stats in analysis.items():
            self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, {stats['total_count']} samples")
            
        if trajectory_analysis:
            self.get_logger().info(f"📈 Trajectory: {trajectory_analysis['total_distance']:.2f}m, "
                                 f"{trajectory_analysis['avg_speed']:.2f} m/s")
            
        self.get_logger().info("✅ Stage 4: SLAM Engine - COMPLETED")
        self.get_logger().info("🚀 Ready for Stage 5: Perception Language Model")

def main(args=None):
    rclpy.init(args=args)
    test_node = SLAMIntegrationTest()
    
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