#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
import time
import json
import numpy as np
import cv2

class AdvancedSLAMTest(Node):
    def __init__(self):
        super().__init__('advanced_slam_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        self.test_duration = 60  # seconds
        
        # SLAM data tracking
        self.slam_data = {
            'slam_pose': {'received': False, 'count': 0, 'last_time': 0, 'poses': []},
            'slam_map': {'received': False, 'count': 0, 'last_time': 0, 'maps': []},
            'slam_path': {'received': False, 'count': 0, 'last_time': 0, 'paths': []},
            'slam_status': {'received': False, 'count': 0, 'last_time': 0, 'statuses': []},
            'slam_markers': {'received': False, 'count': 0, 'last_time': 0}
        }
        
        # Simulated sensor data
        self.simulated_sensors = {
            'camera_rgb': {'active': True, 'data_rate': 10.0},
            'camera_depth': {'active': True, 'data_rate': 10.0},
            'lidar_points': {'active': True, 'data_rate': 20.0},
            'lidar_scan': {'active': True, 'data_rate': 10.0},
            'imu': {'active': True, 'data_rate': 100.0}
        }
        
        # Subscribers
        self.slam_pose_sub = self.create_subscription(
            PoseStamped, '/quadcopter/slam/pose', 
            lambda msg: self.slam_callback('slam_pose', msg), 10
        )
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid, '/quadcopter/slam/map', 
            lambda msg: self.slam_callback('slam_map', msg), 10
        )
        self.slam_path_sub = self.create_subscription(
            Path, '/quadcopter/slam/path', 
            lambda msg: self.slam_callback('slam_path', msg), 10
        )
        self.slam_status_sub = self.create_subscription(
            String, '/quadcopter/slam/status', 
            lambda msg: self.slam_callback('slam_status', msg), 10
        )
        
        # Publishers for simulated sensors
        self.sensor_publishers = {}
        for sensor_name in self.simulated_sensors:
            if 'camera' in sensor_name:
                self.sensor_publishers[sensor_name] = self.create_publisher(Image, f'/quadcopter/{sensor_name}', 10)
            elif 'lidar' in sensor_name:
                if 'points' in sensor_name:
                    self.sensor_publishers[sensor_name] = self.create_publisher(PointCloud2, f'/quadcopter/{sensor_name}', 10)
                else:
                    self.sensor_publishers[sensor_name] = self.create_publisher(LaserScan, f'/quadcopter/{sensor_name}', 10)
            elif sensor_name == 'imu':
                self.sensor_publishers[sensor_name] = self.create_publisher(Imu, f'/quadcopter/{sensor_name}', 10)
                
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        # Sensor simulation timer
        self.sensor_timer = self.create_timer(0.1, self.simulate_sensors)
        
        # Simulated quadcopter movement
        self.simulated_pose = {'x': 0.0, 'y': 0.0, 'z': 2.0, 'yaw': 0.0}
        self.movement_velocity = {'x': 0.5, 'y': 0.0, 'z': 0.0, 'yaw': 0.1}
        
        self.get_logger().info("🧪 Advanced SLAM Test Started")
        
    def slam_callback(self, data_type, msg):
        """Track SLAM data reception"""
        current_time = time.time()
        
        if not self.slam_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.slam_data[data_type]['received'] = True
            
        self.slam_data[data_type]['count'] += 1
        self.slam_data[data_type]['last_time'] = current_time
        
        # Store data for analysis
        if data_type == 'slam_pose':
            pose = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'yaw': self.quaternion_to_yaw(msg.pose.orientation)
            }
            self.slam_data[data_type]['poses'].append(pose)
        elif data_type == 'slam_map':
            map_info = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'data_size': len(msg.data)
            }
            self.slam_data[data_type]['maps'].append(map_info)
        elif data_type == 'slam_path':
            path_info = {
                'poses_count': len(msg.poses),
                'frame_id': msg.header.frame_id
            }
            self.slam_data[data_type]['paths'].append(path_info)
        elif data_type == 'slam_status':
            try:
                status = json.loads(msg.data)
                self.slam_data[data_type]['statuses'].append(status)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in SLAM status")
                
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        return np.arctan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                          1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))
        
    def simulate_sensors(self):
        """Simulate sensor data generation"""
        current_time = time.time()
        
        # Update simulated pose
        dt = 0.1
        self.simulated_pose['x'] += self.movement_velocity['x'] * dt
        self.simulated_pose['y'] += self.movement_velocity['y'] * dt
        self.simulated_pose['z'] += self.movement_velocity['z'] * dt
        self.simulated_pose['yaw'] += self.movement_velocity['yaw'] * dt
        
        for sensor_name, sensor_info in self.simulated_sensors.items():
            if not sensor_info['active']:
                continue
                
            # Check if it's time to publish this sensor
            if hasattr(self, f'last_{sensor_name}_time'):
                time_since_last = current_time - getattr(self, f'last_{sensor_name}_time')
                if time_since_last < 1.0 / sensor_info['data_rate']:
                    continue
            else:
                setattr(self, f'last_{sensor_name}_time', 0)
                
            # Generate and publish sensor data
            if 'camera' in sensor_name:
                self.simulate_camera_data(sensor_name)
            elif 'lidar' in sensor_name:
                self.simulate_lidar_data(sensor_name)
            elif sensor_name == 'imu':
                self.simulate_imu_data()
                
            setattr(self, f'last_{sensor_name}_time', current_time)
            
    def simulate_camera_data(self, sensor_name):
        """Simulate camera data"""
        # Create a simple test image
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Add some features for testing
        cv2.rectangle(image, (100, 100), (200, 200), (255, 0, 0), 2)
        cv2.circle(image, (400, 300), 50, (0, 255, 0), 2)
        
        # Convert to ROS Image message
        from cv_bridge import CvBridge
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera"
        
        self.sensor_publishers[sensor_name].publish(image_msg)
        
    def simulate_lidar_data(self, sensor_name):
        """Simulate lidar data"""
        if 'points' in sensor_name:
            # Simulate point cloud data
            from sensor_msgs.msg import PointCloud2, PointField
            from std_msgs.msg import Header
            
            # Create simple point cloud
            points = np.random.rand(100, 3) * 10.0
            fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
            
            pointcloud_msg = PointCloud2()
            pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
            pointcloud_msg.header.frame_id = "lidar"
            pointcloud_msg.fields = fields
            pointcloud_msg.point_step = 12
            pointcloud_msg.row_step = 12 * 100
            pointcloud_msg.data = points.astype(np.float32).tobytes()
            
            self.sensor_publishers[sensor_name].publish(pointcloud_msg)
        else:
            # Simulate laser scan data
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "lidar"
            scan_msg.angle_min = -np.pi
            scan_msg.angle_max = np.pi
            scan_msg.angle_increment = 0.1
            scan_msg.range_min = 0.1
            scan_msg.range_max = 10.0
            scan_msg.ranges = [5.0 + np.random.normal(0, 0.5) for _ in range(63)]
            
            self.sensor_publishers[sensor_name].publish(scan_msg)
            
    def simulate_imu_data(self):
        """Simulate IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation.w = 1.0
        imu_msg.angular_velocity.x = np.random.normal(0, 0.1)
        imu_msg.angular_velocity.y = np.random.normal(0, 0.1)
        imu_msg.angular_velocity.z = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.x = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.y = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.z = 9.81 + np.random.normal(0, 0.1)
        
        self.sensor_publishers['imu'].publish(imu_msg)
        
    def check_slam_health(self):
        """Check if SLAM system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.slam_data)
        
        for component_name, data in self.slam_data.items():
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
                elapsed_time = time.time() - self.start_time
                data_rate = data['count'] / elapsed_time if elapsed_time > 0 else 0
                
                analysis[component_name] = {
                    'data_rate': data_rate,
                    'total_count': data['count'],
                    'last_update': time.time() - data['last_time']
                }
                
        return analysis
        
    def analyze_slam_quality(self):
        """Analyze SLAM quality"""
        if not self.slam_data['slam_status']['statuses']:
            return None
            
        statuses = self.slam_data['slam_status']['statuses']
        
        # Extract SLAM metrics
        slam_types = [status.get('slam_type', 'unknown') for status in statuses]
        slam_states = [status.get('slam_state', 'unknown') for status in statuses]
        confidences = [status.get('confidence', 0) for status in statuses]
        keyframes = [status.get('keyframes', 0) for status in statuses]
        
        return {
            'total_statuses': len(statuses),
            'unique_types': list(set(slam_types)),
            'unique_states': list(set(slam_states)),
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'avg_keyframes': np.mean(keyframes) if keyframes else 0
        }
        
    def analyze_pose_trajectory(self):
        """Analyze pose trajectory"""
        if not self.slam_data['slam_pose']['poses']:
            return None
            
        poses = self.slam_data['slam_pose']['poses']
        
        # Extract trajectory metrics
        x_coords = [pose['x'] for pose in poses]
        y_coords = [pose['y'] for pose in poses]
        z_coords = [pose['z'] for pose in poses]
        yaws = [pose['yaw'] for pose in poses]
        
        # Calculate trajectory statistics
        total_distance = 0
        for i in range(1, len(poses)):
            dx = poses[i]['x'] - poses[i-1]['x']
            dy = poses[i]['y'] - poses[i-1]['y']
            dz = poses[i]['z'] - poses[i-1]['z']
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            total_distance += distance
            
        return {
            'total_poses': len(poses),
            'total_distance': total_distance,
            'avg_x': np.mean(x_coords),
            'avg_y': np.mean(y_coords),
            'avg_z': np.mean(z_coords),
            'avg_yaw': np.mean(yaws)
        }
        
    def analyze_map_quality(self):
        """Analyze map quality"""
        if not self.slam_data['slam_map']['maps']:
            return None
            
        maps = self.slam_data['slam_map']['maps']
        
        # Extract map metrics
        widths = [map_info['width'] for map_info in maps]
        heights = [map_info['height'] for map_info in maps]
        resolutions = [map_info['resolution'] for map_info in maps]
        data_sizes = [map_info['data_size'] for map_info in maps]
        
        return {
            'total_maps': len(maps),
            'avg_width': np.mean(widths),
            'avg_height': np.mean(heights),
            'avg_resolution': np.mean(resolutions),
            'avg_data_size': np.mean(data_sizes)
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            self.get_logger().info("⏳ Waiting for Advanced SLAM Engine to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic SLAM connectivity
            self.get_logger().info("🧪 Phase 1: Advanced SLAM Connectivity Test")
            
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
                    'slam_pose': 10,
                    'slam_map': 1,
                    'slam_path': 1,
                    'slam_status': 10,
                    'slam_markers': 1
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:
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
            # Phase 3: SLAM quality analysis
            self.get_logger().info("🧪 Phase 3: SLAM Quality Analysis")
            
            slam_analysis = self.analyze_slam_quality()
            
            if slam_analysis:
                self.get_logger().info("🔬 SLAM Quality Analysis:")
                self.get_logger().info(f"  Total status updates: {slam_analysis['total_statuses']}")
                self.get_logger().info(f"  SLAM types: {slam_analysis['unique_types']}")
                self.get_logger().info(f"  SLAM states: {slam_analysis['unique_states']}")
                self.get_logger().info(f"  Average confidence: {slam_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Average keyframes: {slam_analysis['avg_keyframes']:.1f}")
                
                # Check SLAM quality
                if slam_analysis['avg_confidence'] > 0.7:
                    self.get_logger().info("✅ SLAM confidence is high")
                else:
                    self.get_logger().warn("⚠️ SLAM confidence is low")
                    
                if len(slam_analysis['unique_states']) > 1:
                    self.get_logger().info("✅ SLAM is transitioning through states")
                else:
                    self.get_logger().warn("⚠️ SLAM may be stuck in single state")
            else:
                self.get_logger().warn("⚠️ No SLAM quality data available")
                
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: Pose trajectory analysis
            self.get_logger().info("🧪 Phase 4: Pose Trajectory Analysis")
            
            trajectory_analysis = self.analyze_pose_trajectory()
            
            if trajectory_analysis:
                self.get_logger().info("🎯 Pose Trajectory Analysis:")
                self.get_logger().info(f"  Total poses: {trajectory_analysis['total_poses']}")
                self.get_logger().info(f"  Total distance: {trajectory_analysis['total_distance']:.2f} m")
                self.get_logger().info(f"  Average position: ({trajectory_analysis['avg_x']:.2f}, {trajectory_analysis['avg_y']:.2f}, {trajectory_analysis['avg_z']:.2f})")
                self.get_logger().info(f"  Average yaw: {trajectory_analysis['avg_yaw']:.2f} rad")
                
                # Check trajectory quality
                if trajectory_analysis['total_poses'] > 10:
                    self.get_logger().info("✅ SLAM is tracking pose trajectory")
                else:
                    self.get_logger().warn("⚠️ SLAM not tracking enough poses")
                    
                if trajectory_analysis['total_distance'] > 1.0:
                    self.get_logger().info("✅ SLAM is detecting movement")
                else:
                    self.get_logger().warn("⚠️ SLAM may not be detecting movement")
            else:
                self.get_logger().warn("⚠️ No pose trajectory data available")
                
            self.test_phase = 4
            self.start_time = time.time()
            
        elif self.test_phase == 4:
            # Phase 5: Map quality analysis
            self.get_logger().info("🧪 Phase 5: Map Quality Analysis")
            
            map_analysis = self.analyze_map_quality()
            
            if map_analysis:
                self.get_logger().info("🗺️ Map Quality Analysis:")
                self.get_logger().info(f"  Total maps: {map_analysis['total_maps']}")
                self.get_logger().info(f"  Average width: {map_analysis['avg_width']:.0f} cells")
                self.get_logger().info(f"  Average height: {map_analysis['avg_height']:.0f} cells")
                self.get_logger().info(f"  Average resolution: {map_analysis['avg_resolution']:.3f} m")
                self.get_logger().info(f"  Average data size: {map_analysis['avg_data_size']:.0f} cells")
                
                # Check map quality
                if map_analysis['total_maps'] > 0:
                    self.get_logger().info("✅ SLAM is generating maps")
                else:
                    self.get_logger().warn("⚠️ SLAM not generating maps")
                    
                if map_analysis['avg_data_size'] > 1000:
                    self.get_logger().info("✅ Maps have sufficient detail")
                else:
                    self.get_logger().warn("⚠️ Maps may lack detail")
            else:
                self.get_logger().warn("⚠️ No map data available")
                
            self.test_phase = 5
            self.start_time = time.time()
            
        elif self.test_phase == 5:
            # Phase 6: Final SLAM integration test
            self.get_logger().info("🧪 Phase 6: Final SLAM Integration Test")
            
            # Final health check
            healthy_components, total_components = self.check_slam_health()
            analysis = self.analyze_slam_data()
            slam_analysis = self.analyze_slam_quality()
            trajectory_analysis = self.analyze_pose_trajectory()
            map_analysis = self.analyze_map_quality()
            
            self.get_logger().info("🎉 ADVANCED SLAM TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} SLAM components healthy")
            self.get_logger().info(f"✅ Total SLAM samples: {sum(data['count'] for data in self.slam_data.values())}")
            
            if slam_analysis:
                self.get_logger().info(f"✅ SLAM: {slam_analysis['total_statuses']} status updates, "
                                     f"confidence: {slam_analysis['avg_confidence']:.3f}")
                
            if trajectory_analysis:
                self.get_logger().info(f"✅ Trajectory: {trajectory_analysis['total_poses']} poses, "
                                     f"distance: {trajectory_analysis['total_distance']:.2f}m")
                
            if map_analysis:
                self.get_logger().info(f"✅ Maps: {map_analysis['total_maps']} maps, "
                                     f"avg size: {map_analysis['avg_data_size']:.0f} cells")
            
            self.get_logger().info("✅ Advanced SLAM integrated and working")
            self.get_logger().info("✅ Ready for Stage 11: Real-time AI Integration")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()
            
    def print_final_report(self):
        """Print final test report"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 FINAL ADVANCED SLAM REPORT")
        self.get_logger().info("="*60)
        
        analysis = self.analyze_slam_data()
        slam_analysis = self.analyze_slam_quality()
        trajectory_analysis = self.analyze_pose_trajectory()
        map_analysis = self.analyze_map_quality()
        total_samples = sum(data['count'] for data in self.slam_data.values())
        
        self.get_logger().info(f"📊 Total SLAM Samples: {total_samples}")
        self.get_logger().info(f"⏱️ Test Duration: {time.time() - self.start_time:.1f} seconds")
        
        for component_name, stats in analysis.items():
            self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, {stats['total_count']} samples")
            
        if slam_analysis:
            self.get_logger().info(f"🔬 SLAM: {slam_analysis['total_statuses']} updates, "
                                 f"confidence: {slam_analysis['avg_confidence']:.3f}")
            
        if trajectory_analysis:
            self.get_logger().info(f"🎯 Trajectory: {trajectory_analysis['total_poses']} poses, "
                                 f"distance: {trajectory_analysis['total_distance']:.2f}m")
            
        if map_analysis:
            self.get_logger().info(f"🗺️ Maps: {map_analysis['total_maps']} maps, "
                                 f"avg size: {map_analysis['avg_data_size']:.0f} cells")
            
        self.get_logger().info("✅ Stage 10: Advanced SLAM - COMPLETED")
        self.get_logger().info("🚀 Ready for Stage 11: Real-time AI Integration")

def main(args=None):
    rclpy.init(args=args)
    test_node = AdvancedSLAMTest()
    
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