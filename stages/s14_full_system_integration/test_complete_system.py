#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid
import time
import json
import threading
from typing import Dict, List, Optional

class CompleteSystemTester(Node):
    def __init__(self):
        super().__init__('complete_system_tester')
        
        # Test results tracking
        self.test_results = {}
        self.component_status = {}
        self.ai_coordination_data = []
        self.mission_progress = {}
        
        # Test parameters
        self.test_duration = 60  # seconds
        self.test_start_time = time.time()
        
        # Publishers for test commands
        self.test_commands_pub = self.create_publisher(String, '/quadcopter/test/commands', 10)
        
        # Subscribers for all system components
        self.setup_test_subscribers()
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test_cycle)
        
        # Test completion timer
        self.completion_timer = self.create_timer(self.test_duration, self.complete_test)
        
        self.get_logger().info("Complete System Tester initialized")
        
    def setup_test_subscribers(self):
        """Setup subscribers to monitor all system components"""
        
        # Stage 1: Quadcopter Simulation
        self.quadcopter_status_sub = self.create_subscription(
            String, '/quadcopter/status', 
            lambda msg: self.monitor_component('quadcopter_simulation', msg), 10
        )
        
        # Stage 2: 3D Map Creation
        self.map_status_sub = self.create_subscription(
            String, '/quadcopter/map/status', 
            lambda msg: self.monitor_component('3d_map_creation', msg), 10
        )
        
        # Stage 3: Sensor Simulation
        self.sensor_status_sub = self.create_subscription(
            String, '/quadcopter/sensor/status', 
            lambda msg: self.monitor_component('sensor_simulation', msg), 10
        )
        
        # Stage 4: SLAM Engine
        self.slam_status_sub = self.create_subscription(
            String, '/quadcopter/slam/status', 
            lambda msg: self.monitor_component('slam_engine', msg), 10
        )
        
        # Stage 5: Perception Language Model
        self.perception_status_sub = self.create_subscription(
            String, '/quadcopter/perception/status', 
            lambda msg: self.monitor_component('perception_plm', msg), 10
        )
        
        # Stage 6: LLM Decision Engine
        self.decision_status_sub = self.create_subscription(
            String, '/quadcopter/decision/status', 
            lambda msg: self.monitor_component('llm_decision_engine', msg), 10
        )
        
        # Stage 7: Navigation Control
        self.navigation_status_sub = self.create_subscription(
            String, '/quadcopter/navigation/status', 
            lambda msg: self.monitor_component('navigation_control', msg), 10
        )
        
        # Stage 8: Swarm Coordination
        self.swarm_status_sub = self.create_subscription(
            String, '/quadcopter/swarm/status', 
            lambda msg: self.monitor_component('swarm_coordination', msg), 10
        )
        
        # Stage 9: Multi-Sensor Fusion
        self.fusion_status_sub = self.create_subscription(
            String, '/quadcopter/fusion/status', 
            lambda msg: self.monitor_component('multi_sensor_fusion', msg), 10
        )
        
        # Stage 10: Advanced SLAM
        self.advanced_slam_status_sub = self.create_subscription(
            String, '/quadcopter/advanced_slam/status', 
            lambda msg: self.monitor_component('advanced_slam', msg), 10
        )
        
        # Stage 11: Real-time AI Integration
        self.ai_integration_status_sub = self.create_subscription(
            String, '/quadcopter/ai/status', 
            lambda msg: self.monitor_component('realtime_ai_integration', msg), 10
        )
        
        # Stage 12: Mission Planning
        self.mission_status_sub = self.create_subscription(
            String, '/quadcopter/mission/status', 
            lambda msg: self.monitor_component('mission_planning', msg), 10
        )
        
        # Stage 13: Performance Optimization
        self.performance_status_sub = self.create_subscription(
            String, '/quadcopter/performance/status', 
            lambda msg: self.monitor_component('performance_optimization', msg), 10
        )
        
        # Stage 14: Full System Integration
        self.system_status_sub = self.create_subscription(
            String, '/quadcopter/system/status', 
            lambda msg: self.monitor_component('full_system_integration', msg), 10
        )
        
        # AI coordination data
        self.ai_coordination_sub = self.create_subscription(
            String, '/quadcopter/ai/results', 
            self.monitor_ai_coordination, 10
        )
        
        # Mission progress
        self.mission_progress_sub = self.create_subscription(
            String, '/quadcopter/mission/tasks', 
            self.monitor_mission_progress, 10
        )
        
    def monitor_component(self, component_name: str, msg):
        """Monitor individual component status"""
        try:
            status_data = json.loads(msg.data)
            self.component_status[component_name] = {
                'status': status_data.get('status', 'unknown'),
                'timestamp': time.time(),
                'data': status_data
            }
            
            # Log component status
            self.get_logger().info(f"Component {component_name}: {status_data.get('status', 'unknown')}")
            
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON from {component_name}")
            
    def monitor_ai_coordination(self, msg):
        """Monitor AI coordination data"""
        try:
            ai_data = json.loads(msg.data)
            self.ai_coordination_data.append({
                'timestamp': time.time(),
                'data': ai_data
            })
            
            # Keep only recent data
            if len(self.ai_coordination_data) > 100:
                self.ai_coordination_data = self.ai_coordination_data[-100:]
                
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in AI coordination data")
            
    def monitor_mission_progress(self, msg):
        """Monitor mission progress"""
        try:
            mission_data = json.loads(msg.data)
            self.mission_progress = mission_data
            
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in mission progress data")
            
    def run_test_cycle(self):
        """Run one test cycle"""
        current_time = time.time()
        elapsed_time = current_time - self.test_start_time
        
        # Send test commands
        if elapsed_time < 10:  # First 10 seconds
            self.send_test_command('start_mission', {'mission_type': 'exploration'})
        elif elapsed_time < 20:  # 10-20 seconds
            self.send_test_command('test_swarm_formation', {'formation': 'triangle'})
        elif elapsed_time < 30:  # 20-30 seconds
            self.send_test_command('test_ai_coordination', {'test_type': 'perception'})
        elif elapsed_time < 40:  # 30-40 seconds
            self.send_test_command('test_slam_integration', {'test_type': 'mapping'})
        elif elapsed_time < 50:  # 40-50 seconds
            self.send_test_command('test_performance_optimization', {'test_type': 'resource_monitoring'})
            
        # Log test progress
        if int(elapsed_time) % 10 == 0:
            self.log_test_progress()
            
    def send_test_command(self, command_type: str, parameters: dict):
        """Send test command to system"""
        command = {
            'timestamp': time.time(),
            'command_type': command_type,
            'parameters': parameters,
            'test_id': 'complete_system_test'
        }
        
        command_msg = String()
        command_msg.data = json.dumps(command)
        self.test_commands_pub.publish(command_msg)
        
        self.get_logger().info(f"Sent test command: {command_type}")
        
    def log_test_progress(self):
        """Log current test progress"""
        elapsed_time = time.time() - self.test_start_time
        
        # Count active components
        active_components = sum(1 for status in self.component_status.values() 
                              if status.get('status') in ['active', 'healthy', 'ready'])
        
        # Count AI coordination events
        ai_events = len(self.ai_coordination_data)
        
        # Mission status
        mission_status = self.mission_progress.get('status', 'unknown') if self.mission_progress else 'unknown'
        
        self.get_logger().info(f"Test Progress - Time: {elapsed_time:.1f}s, "
                              f"Active Components: {active_components}/14, "
                              f"AI Events: {ai_events}, "
                              f"Mission: {mission_status}")
                              
    def complete_test(self):
        """Complete the test and generate report"""
        self.get_logger().info("=== COMPLETE SYSTEM TEST COMPLETED ===")
        
        # Generate test report
        test_report = self.generate_test_report()
        
        # Log final results
        self.get_logger().info("=== TEST REPORT ===")
        for key, value in test_report.items():
            self.get_logger().info(f"{key}: {value}")
            
        # Save test results
        self.save_test_results(test_report)
        
        # Shutdown
        self.get_logger().info("Test completed. Shutting down...")
        rclpy.shutdown()
        
    def generate_test_report(self):
        """Generate comprehensive test report"""
        elapsed_time = time.time() - self.test_start_time
        
        # Component health analysis
        component_health = {}
        for component_name, status in self.component_status.items():
            component_health[component_name] = {
                'status': status.get('status', 'unknown'),
                'last_update': status.get('timestamp', 0),
                'response_time': elapsed_time - status.get('timestamp', 0) if status.get('timestamp') else float('inf')
            }
            
        # AI coordination analysis
        ai_coordination_summary = {
            'total_events': len(self.ai_coordination_data),
            'event_rate': len(self.ai_coordination_data) / max(elapsed_time, 1),
            'last_event': max([event['timestamp'] for event in self.ai_coordination_data]) if self.ai_coordination_data else 0
        }
        
        # Mission progress analysis
        mission_summary = {
            'status': self.mission_progress.get('status', 'unknown'),
            'progress': self.mission_progress.get('progress', 0),
            'tasks_completed': self.mission_progress.get('tasks_completed', 0)
        }
        
        # Overall system health
        healthy_components = sum(1 for health in component_health.values() 
                               if health['status'] in ['active', 'healthy', 'ready'])
        
        system_health_score = healthy_components / 14.0 if component_health else 0.0
        
        # Test results
        test_results = {
            'test_duration': elapsed_time,
            'system_health_score': system_health_score,
            'healthy_components': healthy_components,
            'total_components': 14,
            'component_health': component_health,
            'ai_coordination': ai_coordination_summary,
            'mission_progress': mission_summary,
            'test_status': 'PASSED' if system_health_score >= 0.8 else 'FAILED',
            'recommendations': self.generate_recommendations(component_health, ai_coordination_summary)
        }
        
        return test_results
        
    def generate_recommendations(self, component_health: dict, ai_coordination: dict) -> List[str]:
        """Generate improvement recommendations"""
        recommendations = []
        
        # Check component health
        unhealthy_components = [name for name, health in component_health.items() 
                              if health['status'] not in ['active', 'healthy', 'ready']]
        if unhealthy_components:
            recommendations.append(f"Fix unhealthy components: {', '.join(unhealthy_components)}")
            
        # Check AI coordination
        if ai_coordination['total_events'] < 10:
            recommendations.append("Increase AI coordination frequency")
            
        # Check response times
        slow_components = [name for name, health in component_health.items() 
                         if health['response_time'] > 5.0]
        if slow_components:
            recommendations.append(f"Optimize slow components: {', '.join(slow_components)}")
            
        if not recommendations:
            recommendations.append("System is performing well")
            
        return recommendations
        
    def save_test_results(self, test_results: dict):
        """Save test results to file"""
        try:
            with open('/tmp/complete_system_test_results.json', 'w') as f:
                json.dump(test_results, f, indent=2)
            self.get_logger().info("Test results saved to /tmp/complete_system_test_results.json")
        except Exception as e:
            self.get_logger().error(f"Failed to save test results: {e}")

def main(args=None):
    rclpy.init(args=args)
    tester = CompleteSystemTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Test stopped by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 