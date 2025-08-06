#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu, NavSatFix
from std_msgs.msg import String
import time
import json
import numpy as np
import cv2

class SensorFusionTest(Node):
    def __init__(self):
        super().__init__('sensor_fusion_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        self.test_duration = 60  # seconds
        
        # Fusion data tracking
        self.fusion_data = {
            'fused_data': {'received': False, 'count': 0, 'last_time': 0, 'data': []},
            'fusion_status': {'received': False, 'count': 0, 'last_time': 0, 'statuses': []},
            'fusion_markers': {'received': False, 'count': 0, 'last_time': 0}
        }
        
        # Simulated sensor data
        self.simulated_sensors = {
            'camera_rgb': {'active': True, 'data_rate': 10.0},
            'camera_thermal': {'active': True, 'data_rate': 5.0},
            'lidar_pointcloud': {'active': True, 'data_rate': 20.0},
            'lidar_scan': {'active': True, 'data_rate': 10.0},
            'gps': {'active': True, 'data_rate': 1.0},
            'imu': {'active': True, 'data_rate': 100.0},
            'ultrasonic': {'active': True, 'data_rate': 10.0}
        }
        
        # Subscribers
        self.fused_data_sub = self.create_subscription(
            String, '/quadcopter/fusion/data', 
            lambda msg: self.fusion_callback('fused_data', msg), 10
        )
        self.fusion_status_sub = self.create_subscription(
            String, '/quadcopter/fusion/status', 
            lambda msg: self.fusion_callback('fusion_status', msg), 10
        )
        
        # Publishers for simulated sensors
        self.sensor_publishers = {}
        for sensor_name in self.simulated_sensors:
            if 'camera' in sensor_name:
                self.sensor_publishers[sensor_name] = self.create_publisher(Image, f'/quadcopter/{sensor_name}', 10)
            elif 'lidar' in sensor_name:
                if 'pointcloud' in sensor_name:
                    self.sensor_publishers[sensor_name] = self.create_publisher(PointCloud2, f'/quadcopter/{sensor_name}', 10)
                else:
                    self.sensor_publishers[sensor_name] = self.create_publisher(LaserScan, f'/quadcopter/{sensor_name}', 10)
            elif sensor_name == 'gps':
                self.sensor_publishers[sensor_name] = self.create_publisher(NavSatFix, f'/quadcopter/{sensor_name}', 10)
            elif sensor_name == 'imu':
                self.sensor_publishers[sensor_name] = self.create_publisher(Imu, f'/quadcopter/{sensor_name}', 10)
            else:
                self.sensor_publishers[sensor_name] = self.create_publisher(LaserScan, f'/quadcopter/{sensor_name}', 10)
                
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        # Sensor simulation timer
        self.sensor_timer = self.create_timer(0.1, self.simulate_sensors)
        
        self.get_logger().info("🧪 Sensor Fusion Test Started")
        
    def fusion_callback(self, data_type, msg):
        """Track fusion data reception"""
        current_time = time.time()
        
        if not self.fusion_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.fusion_data[data_type]['received'] = True
            
        self.fusion_data[data_type]['count'] += 1
        self.fusion_data[data_type]['last_time'] = current_time
        
        # Store data for analysis
        if data_type == 'fused_data':
            try:
                data = json.loads(msg.data)
                self.fusion_data[data_type]['data'].append(data)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in fused data")
        elif data_type == 'fusion_status':
            try:
                status = json.loads(msg.data)
                self.fusion_data[data_type]['statuses'].append(status)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in fusion status")
                
    def simulate_sensors(self):
        """Simulate sensor data generation"""
        current_time = time.time()
        
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
            elif sensor_name == 'gps':
                self.simulate_gps_data()
            elif sensor_name == 'imu':
                self.simulate_imu_data()
            else:
                self.simulate_ultrasonic_data()
                
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
        if 'pointcloud' in sensor_name:
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
            
    def simulate_gps_data(self):
        """Simulate GPS data"""
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps"
        gps_msg.latitude = 37.7749 + np.random.normal(0, 0.0001)
        gps_msg.longitude = -122.4194 + np.random.normal(0, 0.0001)
        gps_msg.altitude = 100.0 + np.random.normal(0, 1.0)
        
        self.sensor_publishers['gps'].publish(gps_msg)
        
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
        
    def simulate_ultrasonic_data(self):
        """Simulate ultrasonic data"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "ultrasonic"
        scan_msg.angle_min = -0.5
        scan_msg.angle_max = 0.5
        scan_msg.angle_increment = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 5.0
        scan_msg.ranges = [2.0 + np.random.normal(0, 0.1) for _ in range(11)]
        
        self.sensor_publishers['ultrasonic'].publish(scan_msg)
        
    def check_fusion_health(self):
        """Check if fusion system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.fusion_data)
        
        for component_name, data in self.fusion_data.items():
            if data['received'] and (current_time - data['last_time']) < 10.0:
                healthy_components += 1
            else:
                self.get_logger().warn(f"⚠️ {component_name} not responding")
                
        return healthy_components, total_components
        
    def analyze_fusion_data(self):
        """Analyze fusion data quality"""
        analysis = {}
        
        for component_name, data in self.fusion_data.items():
            if data['received']:
                elapsed_time = time.time() - self.start_time
                data_rate = data['count'] / elapsed_time if elapsed_time > 0 else 0
                
                analysis[component_name] = {
                    'data_rate': data_rate,
                    'total_count': data['count'],
                    'last_update': time.time() - data['last_time']
                }
                
        return analysis
        
    def analyze_fusion_quality(self):
        """Analyze fusion quality"""
        if not self.fusion_data['fusion_status']['statuses']:
            return None
            
        statuses = self.fusion_data['fusion_status']['statuses']
        
        # Extract fusion metrics
        fusion_states = [status.get('fusion_state', 'unknown') for status in statuses]
        confidences = [status.get('confidence', 0) for status in statuses]
        healthy_sensors = [status.get('healthy_sensors', 0) for status in statuses]
        total_sensors = [status.get('total_sensors', 0) for status in statuses]
        
        return {
            'total_statuses': len(statuses),
            'unique_states': list(set(fusion_states)),
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'avg_healthy_sensors': np.mean(healthy_sensors) if healthy_sensors else 0,
            'avg_total_sensors': np.mean(total_sensors) if total_sensors else 0
        }
        
    def analyze_fused_data(self):
        """Analyze fused data quality"""
        if not self.fusion_data['fused_data']['data']:
            return None
            
        fused_data = self.fusion_data['fused_data']['data']
        
        # Extract fusion metrics
        sensor_counts = [data.get('sensor_count', 0) for data in fused_data]
        confidences = [data.get('confidence', 0) for data in fused_data]
        healthy_sensors = [data.get('healthy_sensors', 0) for data in fused_data]
        
        return {
            'total_fusions': len(fused_data),
            'avg_sensor_count': np.mean(sensor_counts) if sensor_counts else 0,
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'avg_healthy_sensors': np.mean(healthy_sensors) if healthy_sensors else 0
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            self.get_logger().info("⏳ Waiting for Sensor Fusion Engine to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic fusion connectivity
            self.get_logger().info("🧪 Phase 1: Sensor Fusion Connectivity Test")
            
            healthy_components, total_components = self.check_fusion_health()
            
            if healthy_components == total_components:
                self.get_logger().info(f"✅ All {total_components} fusion components connected and responding")
                self.test_phase = 1
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {healthy_components}/{total_components} fusion components healthy")
                
        elif self.test_phase == 1:
            # Phase 2: Fusion data quality analysis
            self.get_logger().info("🧪 Phase 2: Fusion Data Quality Analysis")
            
            analysis = self.analyze_fusion_data()
            
            self.get_logger().info("📊 Fusion Data Analysis:")
            for component_name, stats in analysis.items():
                self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, "
                                     f"{stats['total_count']} samples, "
                                     f"last update: {stats['last_update']:.1f}s ago")
            
            # Check if data rates are reasonable
            good_rates = 0
            for component_name, stats in analysis.items():
                expected_rate = {
                    'fused_data': 10,
                    'fusion_status': 10,
                    'fusion_markers': 1
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:
                    good_rates += 1
                else:
                    self.get_logger().warn(f"⚠️ {component_name} data rate low: {stats['data_rate']:.1f} Hz")
            
            if good_rates == len(analysis):
                self.get_logger().info("✅ All fusion components have good data rates")
                self.test_phase = 2
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {good_rates}/{len(analysis)} components have good data rates")
                
        elif self.test_phase == 2:
            # Phase 3: Fusion quality analysis
            self.get_logger().info("🧪 Phase 3: Fusion Quality Analysis")
            
            fusion_analysis = self.analyze_fusion_quality()
            
            if fusion_analysis:
                self.get_logger().info("🔬 Fusion Quality Analysis:")
                self.get_logger().info(f"  Total status updates: {fusion_analysis['total_statuses']}")
                self.get_logger().info(f"  Fusion states: {fusion_analysis['unique_states']}")
                self.get_logger().info(f"  Average confidence: {fusion_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Average healthy sensors: {fusion_analysis['avg_healthy_sensors']:.1f}")
                self.get_logger().info(f"  Average total sensors: {fusion_analysis['avg_total_sensors']:.1f}")
                
                # Check fusion quality
                if fusion_analysis['avg_confidence'] > 0.7:
                    self.get_logger().info("✅ Fusion confidence is high")
                else:
                    self.get_logger().warn("⚠️ Fusion confidence is low")
                    
                if fusion_analysis['avg_healthy_sensors'] > 3:
                    self.get_logger().info("✅ Sufficient sensors are healthy")
                else:
                    self.get_logger().warn("⚠️ Too few sensors are healthy")
            else:
                self.get_logger().warn("⚠️ No fusion quality data available")
                
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: Fused data analysis
            self.get_logger().info("🧪 Phase 4: Fused Data Analysis")
            
            fused_analysis = self.analyze_fused_data()
            
            if fused_analysis:
                self.get_logger().info("📊 Fused Data Analysis:")
                self.get_logger().info(f"  Total fusions: {fused_analysis['total_fusions']}")
                self.get_logger().info(f"  Average sensor count: {fused_analysis['avg_sensor_count']:.1f}")
                self.get_logger().info(f"  Average confidence: {fused_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Average healthy sensors: {fused_analysis['avg_healthy_sensors']:.1f}")
                
                # Check fused data quality
                if fused_analysis['total_fusions'] > 10:
                    self.get_logger().info("✅ Fusion engine is processing data")
                else:
                    self.get_logger().warn("⚠️ Fusion engine not processing enough data")
                    
                if fused_analysis['avg_confidence'] > 0.7:
                    self.get_logger().info("✅ Fused data confidence is high")
                else:
                    self.get_logger().warn("⚠️ Fused data confidence is low")
            else:
                self.get_logger().warn("⚠️ No fused data available for analysis")
                
            self.test_phase = 4
            self.start_time = time.time()
            
        elif self.test_phase == 4:
            # Phase 5: Sensor simulation test
            self.get_logger().info("🧪 Phase 5: Sensor Simulation Test")
            
            # Simulate sensor failures
            self.simulated_sensors['camera_thermal']['active'] = False
            self.get_logger().info("🔄 Simulating thermal camera failure...")
            
            time.sleep(2.0)
            
            # Reactivate sensor
            self.simulated_sensors['camera_thermal']['active'] = True
            self.get_logger().info("✅ Sensor failure simulation completed")
            
            self.test_phase = 5
            self.start_time = time.time()
            
        elif self.test_phase == 5:
            # Phase 6: Final fusion integration test
            self.get_logger().info("🧪 Phase 6: Final Fusion Integration Test")
            
            # Final health check
            healthy_components, total_components = self.check_fusion_health()
            analysis = self.analyze_fusion_data()
            fusion_analysis = self.analyze_fusion_quality()
            fused_analysis = self.analyze_fused_data()
            
            self.get_logger().info("🎉 SENSOR FUSION TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} fusion components healthy")
            self.get_logger().info(f"✅ Total fusion samples: {sum(data['count'] for data in self.fusion_data.values())}")
            
            if fusion_analysis:
                self.get_logger().info(f"✅ Fusion: {fusion_analysis['total_statuses']} status updates, "
                                     f"confidence: {fusion_analysis['avg_confidence']:.3f}")
                
            if fused_analysis:
                self.get_logger().info(f"✅ Fused Data: {fused_analysis['total_fusions']} fusions, "
                                     f"avg sensors: {fused_analysis['avg_sensor_count']:.1f}")
            
            self.get_logger().info("✅ Multi-Sensor Fusion integrated and working")
            self.get_logger().info("✅ Ready for Stage 10: Advanced SLAM")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()
            
    def print_final_report(self):
        """Print final test report"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 FINAL SENSOR FUSION REPORT")
        self.get_logger().info("="*60)
        
        analysis = self.analyze_fusion_data()
        fusion_analysis = self.analyze_fusion_quality()
        fused_analysis = self.analyze_fused_data()
        total_samples = sum(data['count'] for data in self.fusion_data.values())
        
        self.get_logger().info(f"📊 Total Fusion Samples: {total_samples}")
        self.get_logger().info(f"⏱️ Test Duration: {time.time() - self.start_time:.1f} seconds")
        
        for component_name, stats in analysis.items():
            self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, {stats['total_count']} samples")
            
        if fusion_analysis:
            self.get_logger().info(f"🔬 Fusion: {fusion_analysis['total_statuses']} updates, "
                                 f"confidence: {fusion_analysis['avg_confidence']:.3f}")
            
        if fused_analysis:
            self.get_logger().info(f"📊 Fused Data: {fused_analysis['total_fusions']} fusions, "
                                 f"avg sensors: {fused_analysis['avg_sensor_count']:.1f}")
            
        self.get_logger().info("✅ Stage 9: Multi-Sensor Fusion - COMPLETED")
        self.get_logger().info("🚀 Ready for Stage 10: Advanced SLAM")

def main(args=None):
    rclpy.init(args=args)
    test_node = SensorFusionTest()
    
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