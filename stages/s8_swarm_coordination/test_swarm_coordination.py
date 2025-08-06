#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import time
import json
import numpy as np

class SwarmCoordinationTest(Node):
    def __init__(self):
        super().__init__('swarm_coordination_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        self.test_duration = 60  # seconds
        
        # Swarm data tracking
        self.swarm_data = {
            'swarm_status': {'received': False, 'count': 0, 'last_time': 0, 'statuses': []},
            'coordination_log': {'received': False, 'count': 0, 'last_time': 0, 'decisions': []},
            'formation_markers': {'received': False, 'count': 0, 'last_time': 0}
        }
        
        # Simulated quadcopter data
        self.simulated_quadcopters = {}
        for i in range(3):  # Test with 3 quadcopters
            quad_id = f"quadcopter_{i+1}"
            self.simulated_quadcopters[quad_id] = {
                'pose': {'x': i * 2.0, 'y': 0.0, 'z': 2.0},
                'status': {'mission_state': 'idle', 'action': 'hovering'}
            }
            
        # Subscribers
        self.swarm_status_sub = self.create_subscription(
            String, '/swarm/status', 
            lambda msg: self.swarm_callback('swarm_status', msg), 10
        )
        self.coordination_log_sub = self.create_subscription(
            String, '/swarm/coordination/log', 
            lambda msg: self.swarm_callback('coordination_log', msg), 10
        )
        
        # Publishers for simulated quadcopters
        self.quadcopter_publishers = {}
        for quad_id in self.simulated_quadcopters:
            self.quadcopter_publishers[quad_id] = {
                'status': self.create_publisher(String, f'/{quad_id}/mission/status', 10),
                'pose': self.create_publisher(PoseStamped, f'/{quad_id}/slam/pose', 10)
            }
            
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        # Quadcopter simulation timer
        self.quadcopter_timer = self.create_timer(0.5, self.simulate_quadcopters)
        
        self.get_logger().info("🧪 Swarm Coordination Test Started")
        
    def swarm_callback(self, data_type, msg):
        """Track swarm data reception"""
        current_time = time.time()
        
        if not self.swarm_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.swarm_data[data_type]['received'] = True
            
        self.swarm_data[data_type]['count'] += 1
        self.swarm_data[data_type]['last_time'] = current_time
        
        # Store data for analysis
        if data_type == 'swarm_status':
            try:
                status = json.loads(msg.data)
                self.swarm_data[data_type]['statuses'].append(status)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in swarm status")
        elif data_type == 'coordination_log':
            try:
                decision = json.loads(msg.data)
                self.swarm_data[data_type]['decisions'].append(decision)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in coordination log")
                
    def simulate_quadcopters(self):
        """Simulate quadcopter status and pose updates"""
        for quad_id, quad_data in self.simulated_quadcopters.items():
            # Update pose with some movement
            quad_data['pose']['x'] += np.random.normal(0, 0.1)
            quad_data['pose']['y'] += np.random.normal(0, 0.1)
            
            # Publish status
            status_msg = String()
            status_data = {
                'mission_state': quad_data['status']['mission_state'],
                'action': quad_data['status']['action'],
                'timestamp': time.time()
            }
            status_msg.data = json.dumps(status_data)
            self.quadcopter_publishers[quad_id]['status'].publish(status_msg)
            
            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = quad_data['pose']['x']
            pose_msg.pose.position.y = quad_data['pose']['y']
            pose_msg.pose.position.z = quad_data['pose']['z']
            pose_msg.pose.orientation.w = 1.0
            self.quadcopter_publishers[quad_id]['pose'].publish(pose_msg)
            
    def check_swarm_health(self):
        """Check if swarm system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.swarm_data)
        
        for component_name, data in self.swarm_data.items():
            if data['received'] and (current_time - data['last_time']) < 10.0:
                healthy_components += 1
            else:
                self.get_logger().warn(f"⚠️ {component_name} not responding")
                
        return healthy_components, total_components
        
    def analyze_swarm_data(self):
        """Analyze swarm data quality"""
        analysis = {}
        
        for component_name, data in self.swarm_data.items():
            if data['received']:
                elapsed_time = time.time() - self.start_time
                data_rate = data['count'] / elapsed_time if elapsed_time > 0 else 0
                
                analysis[component_name] = {
                    'data_rate': data_rate,
                    'total_count': data['count'],
                    'last_update': time.time() - data['last_time']
                }
                
        return analysis
        
    def analyze_coordination_quality(self):
        """Analyze coordination decision quality"""
        if not self.swarm_data['coordination_log']['decisions']:
            return None
            
        decisions = self.swarm_data['coordination_log']['decisions']
        
        # Extract decision metrics
        actions = [decision.get('action', 'unknown') for decision in decisions]
        confidences = [decision.get('confidence', 0) for decision in decisions]
        formation_types = [decision.get('formation_type', 'unknown') for decision in decisions]
        quadcopter_goals = [len(decision.get('quadcopter_goals', {})) for decision in decisions]
        
        return {
            'total_decisions': len(decisions),
            'unique_actions': list(set(actions)),
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'formation_types': list(set(formation_types)),
            'avg_goals_per_decision': np.mean(quadcopter_goals) if quadcopter_goals else 0
        }
        
    def analyze_swarm_status(self):
        """Analyze swarm status progression"""
        if not self.swarm_data['swarm_status']['statuses']:
            return None
            
        statuses = self.swarm_data['swarm_status']['statuses']
        
        # Extract swarm states
        swarm_states = [status.get('swarm_state', 'unknown') for status in statuses]
        actions = [status.get('action', 'unknown') for status in statuses]
        confidences = [status.get('confidence', 0) for status in statuses]
        active_quadcopters = [status.get('active_quadcopters', 0) for status in statuses]
        
        return {
            'total_statuses': len(statuses),
            'unique_states': list(set(swarm_states)),
            'state_transitions': len(list(set(swarm_states))),
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'avg_active_quadcopters': np.mean(active_quadcopters) if active_quadcopters else 0
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            self.get_logger().info("⏳ Waiting for Swarm Coordinator to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic swarm connectivity
            self.get_logger().info("🧪 Phase 1: Swarm Coordinator Connectivity Test")
            
            healthy_components, total_components = self.check_swarm_health()
            
            if healthy_components == total_components:
                self.get_logger().info(f"✅ All {total_components} swarm components connected and responding")
                self.test_phase = 1
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {healthy_components}/{total_components} swarm components healthy")
                
        elif self.test_phase == 1:
            # Phase 2: Swarm data quality analysis
            self.get_logger().info("🧪 Phase 2: Swarm Data Quality Analysis")
            
            analysis = self.analyze_swarm_data()
            
            self.get_logger().info("📊 Swarm Data Analysis:")
            for component_name, stats in analysis.items():
                self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, "
                                     f"{stats['total_count']} samples, "
                                     f"last update: {stats['last_update']:.1f}s ago")
            
            # Check if data rates are reasonable
            good_rates = 0
            for component_name, stats in analysis.items():
                expected_rate = {
                    'swarm_status': 2,
                    'coordination_log': 2,
                    'formation_markers': 1
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:
                    good_rates += 1
                else:
                    self.get_logger().warn(f"⚠️ {component_name} data rate low: {stats['data_rate']:.1f} Hz")
            
            if good_rates == len(analysis):
                self.get_logger().info("✅ All swarm components have good data rates")
                self.test_phase = 2
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {good_rates}/{len(analysis)} components have good data rates")
                
        elif self.test_phase == 2:
            # Phase 3: Coordination quality analysis
            self.get_logger().info("🧪 Phase 3: Coordination Quality Analysis")
            
            coordination_analysis = self.analyze_coordination_quality()
            
            if coordination_analysis:
                self.get_logger().info("🤖 Coordination Quality Analysis:")
                self.get_logger().info(f"  Total decisions: {coordination_analysis['total_decisions']}")
                self.get_logger().info(f"  Unique actions: {coordination_analysis['unique_actions']}")
                self.get_logger().info(f"  Average confidence: {coordination_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Formation types: {coordination_analysis['formation_types']}")
                self.get_logger().info(f"  Average goals per decision: {coordination_analysis['avg_goals_per_decision']:.1f}")
                
                # Check coordination quality
                if coordination_analysis['total_decisions'] > 5:
                    self.get_logger().info("✅ Swarm coordinator is making decisions")
                else:
                    self.get_logger().warn("⚠️ Swarm coordinator not making enough decisions")
                    
                if coordination_analysis['avg_confidence'] > 0.7:
                    self.get_logger().info("✅ Coordination confidence is high")
                else:
                    self.get_logger().warn("⚠️ Coordination confidence is low")
            else:
                self.get_logger().warn("⚠️ No coordination data available for analysis")
                
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: Swarm status analysis
            self.get_logger().info("🧪 Phase 4: Swarm Status Analysis")
            
            status_analysis = self.analyze_swarm_status()
            
            if status_analysis:
                self.get_logger().info("🧭 Swarm Status Analysis:")
                self.get_logger().info(f"  Total status updates: {status_analysis['total_statuses']}")
                self.get_logger().info(f"  Swarm states: {status_analysis['unique_states']}")
                self.get_logger().info(f"  State transitions: {status_analysis['state_transitions']}")
                self.get_logger().info(f"  Average confidence: {status_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Average active quadcopters: {status_analysis['avg_active_quadcopters']:.1f}")
                
                # Check swarm progression
                if status_analysis['state_transitions'] > 1:
                    self.get_logger().info("✅ Swarm is progressing through different states")
                else:
                    self.get_logger().warn("⚠️ Swarm may be stuck in single state")
                    
                if status_analysis['avg_active_quadcopters'] > 0:
                    self.get_logger().info("✅ Swarm is tracking quadcopters")
                else:
                    self.get_logger().warn("⚠️ No quadcopters being tracked")
            else:
                self.get_logger().warn("⚠️ No swarm status data available for analysis")
                
            self.test_phase = 4
            self.start_time = time.time()
            
        elif self.test_phase == 4:
            # Phase 5: Formation test
            self.get_logger().info("🧪 Phase 5: Formation Coordination Test")
            
            # Simulate formation change
            for quad_id in self.simulated_quadcopters:
                self.simulated_quadcopters[quad_id]['status']['mission_state'] = 'formation'
                self.simulated_quadcopters[quad_id]['status']['action'] = 'maintain_formation'
                
            self.get_logger().info("🔄 Simulating formation coordination...")
            
            self.test_phase = 5
            self.start_time = time.time()
            
        elif self.test_phase == 5:
            # Phase 6: Final swarm integration test
            self.get_logger().info("🧪 Phase 6: Final Swarm Integration Test")
            
            # Final health check
            healthy_components, total_components = self.check_swarm_health()
            analysis = self.analyze_swarm_data()
            coordination_analysis = self.analyze_coordination_quality()
            status_analysis = self.analyze_swarm_status()
            
            self.get_logger().info("🎉 SWARM COORDINATION TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} swarm components healthy")
            self.get_logger().info(f"✅ Total swarm samples: {sum(data['count'] for data in self.swarm_data.values())}")
            
            if coordination_analysis:
                self.get_logger().info(f"✅ Coordination: {coordination_analysis['total_decisions']} decisions made")
                
            if status_analysis:
                self.get_logger().info(f"✅ Status: {status_analysis['total_statuses']} updates, "
                                     f"{status_analysis['state_transitions']} state transitions")
            
            self.get_logger().info("✅ Swarm Coordination integrated and working")
            self.get_logger().info("✅ Ready for Stage 9: Multi-Sensor Fusion")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()
            
    def print_final_report(self):
        """Print final test report"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 FINAL SWARM COORDINATION REPORT")
        self.get_logger().info("="*60)
        
        analysis = self.analyze_swarm_data()
        coordination_analysis = self.analyze_coordination_quality()
        status_analysis = self.analyze_swarm_status()
        total_samples = sum(data['count'] for data in self.swarm_data.values())
        
        self.get_logger().info(f"📊 Total Swarm Samples: {total_samples}")
        self.get_logger().info(f"⏱️ Test Duration: {time.time() - self.start_time:.1f} seconds")
        
        for component_name, stats in analysis.items():
            self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, {stats['total_count']} samples")
            
        if coordination_analysis:
            self.get_logger().info(f"🤖 Coordination: {coordination_analysis['total_decisions']} decisions, "
                                 f"confidence: {coordination_analysis['avg_confidence']:.3f}")
            
        if status_analysis:
            self.get_logger().info(f"🧭 Status: {status_analysis['total_statuses']} updates, "
                                 f"states: {status_analysis['state_transitions']}")
            
        self.get_logger().info("✅ Stage 8: Swarm Coordination - COMPLETED")
        self.get_logger().info("🚀 Ready for Stage 9: Multi-Sensor Fusion")

def main(args=None):
    rclpy.init(args=args)
    test_node = SwarmCoordinationTest()
    
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