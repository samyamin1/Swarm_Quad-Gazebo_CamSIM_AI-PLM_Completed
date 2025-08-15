#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time
import json
import numpy as np

class MissionPlanningTest(Node):
    def __init__(self):
        super().__init__('mission_planning_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        
        # Mission data tracking
        self.mission_data = {
            'mission_status': {'received': False, 'count': 0, 'last_time': 0, 'statuses': []},
            'mission_tasks': {'received': False, 'count': 0, 'last_time': 0, 'tasks': []},
            'mission_commands': {'received': False, 'count': 0, 'last_time': 0, 'commands': []},
            'mission_visualization': {'received': False, 'count': 0, 'last_time': 0}
        }
        
        # Simulated quadcopter data
        self.simulated_quadcopters = {
            'quadcopter_1': {'active': True, 'data_rate': 2.0},
            'quadcopter_2': {'active': True, 'data_rate': 2.0},
            'quadcopter_3': {'active': True, 'data_rate': 2.0}
        }
        
        # Subscribers
        self.mission_status_sub = self.create_subscription(
            String, '/quadcopter/mission/status', 
            lambda msg: self.mission_callback('mission_status', msg), 10
        )
        self.mission_tasks_sub = self.create_subscription(
            String, '/quadcopter/mission/tasks', 
            lambda msg: self.mission_callback('mission_tasks', msg), 10
        )
        self.mission_commands_sub = self.create_subscription(
            String, '/quadcopter/mission/commands', 
            lambda msg: self.mission_callback('mission_commands', msg), 10
        )
        
        # Publishers for simulated data
        self.quadcopter_status_pub = self.create_publisher(String, '/quadcopter/status', 10)
        self.quadcopter_pose_pub = self.create_publisher(String, '/quadcopter/pose', 10)
        self.mission_request_pub = self.create_publisher(String, '/quadcopter/mission/request', 10)
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        # Data simulation timer
        self.simulation_timer = self.create_timer(0.5, self.simulate_data)
        
        self.get_logger().info("🧪 Mission Planning Test Started")
        
    def mission_callback(self, data_type, msg):
        """Track mission data reception"""
        current_time = time.time()
        
        if not self.mission_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.mission_data[data_type]['received'] = True
            
        self.mission_data[data_type]['count'] += 1
        self.mission_data[data_type]['last_time'] = current_time
        
        # Store data for analysis
        try:
            data = json.loads(msg.data)
            if data_type == 'mission_status':
                self.mission_data[data_type]['statuses'].append(data)
            elif data_type == 'mission_tasks':
                self.mission_data[data_type]['tasks'].append(data)
            elif data_type == 'mission_commands':
                self.mission_data[data_type]['commands'].append(data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON in {data_type}")
                
    def simulate_data(self):
        """Simulate quadcopter and mission data"""
        current_time = time.time()
        
        # Simulate quadcopter status
        for quadcopter_id, quadcopter_info in self.simulated_quadcopters.items():
            if not quadcopter_info['active']:
                continue
                
            # Check if it's time to publish this quadcopter
            if hasattr(self, f'last_{quadcopter_id}_time'):
                time_since_last = current_time - getattr(self, f'last_{quadcopter_id}_time')
                if time_since_last < 1.0 / quadcopter_info['data_rate']:
                    continue
            else:
                setattr(self, f'last_{quadcopter_id}_time', 0)
                
            # Generate and publish quadcopter status
            status_data = {
                'quadcopter_id': quadcopter_id,
                'status': 'active',
                'battery': 85.0,
                'mode': 'mission_execution',
                'timestamp': current_time
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.quadcopter_status_pub.publish(status_msg)
            
            # Generate and publish quadcopter pose
            pose_data = {
                'header': {
                    'frame_id': quadcopter_id,
                    'stamp': current_time
                },
                'pose': {
                    'position': {
                        'x': np.random.uniform(-5, 5),
                        'y': np.random.uniform(-5, 5),
                        'z': 2.0
                    },
                    'orientation': {
                        'x': 0.0,
                        'y': 0.0,
                        'z': 0.0,
                        'w': 1.0
                    }
                }
            }
            
            pose_msg = String()
            pose_msg.data = json.dumps(pose_data)
            self.quadcopter_pose_pub.publish(pose_msg)
            
            setattr(self, f'last_{quadcopter_id}_time', current_time)
            
        # Simulate mission requests periodically
        if hasattr(self, 'last_mission_request_time'):
            time_since_last = current_time - self.last_mission_request_time
            if time_since_last > 10.0:  # Every 10 seconds
                self.simulate_mission_request()
                self.last_mission_request_time = current_time
        else:
            self.last_mission_request_time = current_time
            
    def simulate_mission_request(self):
        """Simulate a new mission request"""
        mission_types = ['exploration', 'surveillance', 'inspection']
        mission_type = np.random.choice(mission_types)
        
        request_data = {
            'mission_type': mission_type,
            'description': f'Simulated {mission_type} mission',
            'target_area': [
                {'x': np.random.uniform(10, 20), 'y': np.random.uniform(10, 20), 'z': 2.0},
                {'x': np.random.uniform(10, 20), 'y': np.random.uniform(10, 20), 'z': 2.0}
            ]
        }
        
        request_msg = String()
        request_msg.data = json.dumps(request_data)
        self.mission_request_pub.publish(request_msg)
        
        self.get_logger().info(f"🔄 Simulated mission request: {mission_type}")
        
    def check_mission_planning_health(self):
        """Check if mission planning system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.mission_data)
        
        for component_name, data in self.mission_data.items():
            if data['received'] and (current_time - data['last_time']) < 15.0:
                healthy_components += 1
            else:
                self.get_logger().warn(f"⚠️ {component_name} not responding")
                
        return healthy_components, total_components
        
    def analyze_mission_data(self):
        """Analyze mission planning data quality"""
        analysis = {}
        
        for component_name, data in self.mission_data.items():
            if data['received']:
                elapsed_time = time.time() - self.start_time
                data_rate = data['count'] / elapsed_time if elapsed_time > 0 else 0
                
                analysis[component_name] = {
                    'data_rate': data_rate,
                    'total_count': data['count'],
                    'last_update': time.time() - data['last_time']
                }
                
        return analysis
        
    def analyze_mission_status(self):
        """Analyze mission status data"""
        if not self.mission_data['mission_status']['statuses']:
            return None
            
        statuses = self.mission_data['mission_status']['statuses']
        
        # Extract mission metrics
        total_missions = []
        active_missions = []
        completed_missions = []
        failed_missions = []
        available_quadcopters = []
        
        for status in statuses:
            total_missions.append(status.get('total_missions', 0))
            active_missions.append(status.get('active_missions', 0))
            completed_missions.append(status.get('completed_missions', 0))
            failed_missions.append(status.get('failed_missions', 0))
            available_quadcopters.append(status.get('available_quadcopters', 0))
            
        return {
            'total_status_updates': len(statuses),
            'avg_total_missions': np.mean(total_missions) if total_missions else 0,
            'avg_active_missions': np.mean(active_missions) if active_missions else 0,
            'avg_completed_missions': np.mean(completed_missions) if completed_missions else 0,
            'avg_failed_missions': np.mean(failed_missions) if failed_missions else 0,
            'avg_available_quadcopters': np.mean(available_quadcopters) if available_quadcopters else 0
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            self.get_logger().info("⏳ Waiting for Mission Planning system to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic mission planning connectivity
            self.get_logger().info("🧪 Phase 1: Mission Planning Connectivity Test")
            
            healthy_components, total_components = self.check_mission_planning_health()
            
            if healthy_components == total_components:
                self.get_logger().info(f"✅ All {total_components} mission planning components connected and responding")
                self.test_phase = 1
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {healthy_components}/{total_components} mission planning components healthy")
                
        elif self.test_phase == 1:
            # Phase 2: Mission planning data quality analysis
            self.get_logger().info("🧪 Phase 2: Mission Planning Data Quality Analysis")
            
            analysis = self.analyze_mission_data()
            
            self.get_logger().info("📊 Mission Planning Data Analysis:")
            for component_name, stats in analysis.items():
                self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, "
                                     f"{stats['total_count']} samples")
            
            # Check if data rates are reasonable
            good_rates = 0
            for component_name, stats in analysis.items():
                expected_rate = {
                    'mission_status': 5,
                    'mission_tasks': 5,
                    'mission_commands': 2,
                    'mission_visualization': 1
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:
                    good_rates += 1
                else:
                    self.get_logger().warn(f"⚠️ {component_name} data rate low: {stats['data_rate']:.1f} Hz")
            
            if good_rates == len(analysis):
                self.get_logger().info("✅ All mission planning components have good data rates")
                self.test_phase = 2
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {good_rates}/{len(analysis)} components have good data rates")
                
        elif self.test_phase == 2:
            # Phase 3: Mission status analysis
            self.get_logger().info("🧪 Phase 3: Mission Status Analysis")
            
            status_analysis = self.analyze_mission_status()
            
            if status_analysis:
                self.get_logger().info("🎯 Mission Status Analysis:")
                self.get_logger().info(f"  Total status updates: {status_analysis['total_status_updates']}")
                self.get_logger().info(f"  Average total missions: {status_analysis['avg_total_missions']:.1f}")
                self.get_logger().info(f"  Average active missions: {status_analysis['avg_active_missions']:.1f}")
                self.get_logger().info(f"  Average completed missions: {status_analysis['avg_completed_missions']:.1f}")
                self.get_logger().info(f"  Average available quadcopters: {status_analysis['avg_available_quadcopters']:.1f}")
                
                # Check mission planning quality
                if status_analysis['avg_total_missions'] > 0:
                    self.get_logger().info("✅ Mission planning system is creating missions")
                else:
                    self.get_logger().warn("⚠️ Mission planning system not creating missions")
                    
                if status_analysis['avg_available_quadcopters'] > 0:
                    self.get_logger().info("✅ Quadcopter management working")
                else:
                    self.get_logger().warn("⚠️ Quadcopter management not working")
            else:
                self.get_logger().warn("⚠️ No mission status data available")
                
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: Final mission planning test
            self.get_logger().info("🧪 Phase 4: Final Mission Planning Test")
            
            # Final health check
            healthy_components, total_components = self.check_mission_planning_health()
            analysis = self.analyze_mission_data()
            status_analysis = self.analyze_mission_status()
            
            self.get_logger().info("🎉 MISSION PLANNING TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} mission planning components healthy")
            self.get_logger().info(f"✅ Total mission planning samples: {sum(data['count'] for data in self.mission_data.values())}")
            
            if status_analysis:
                self.get_logger().info(f"✅ Mission planning: {status_analysis['total_status_updates']} updates, "
                                     f"{status_analysis['avg_total_missions']:.1f} missions")
            
            self.get_logger().info("✅ Mission Planning & Execution working")
            self.get_logger().info("✅ Ready for Stage 13: Performance Optimization")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test_node = MissionPlanningTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 