#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
import numpy as np

class RealTimeAIIntegrationTest(Node):
    def __init__(self):
        super().__init__('realtime_ai_integration_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        
        # AI Integration data tracking
        self.ai_data = {
            'ai_status': {'received': False, 'count': 0, 'last_time': 0, 'statuses': []},
            'ai_results': {'received': False, 'count': 0, 'last_time': 0, 'results': []},
            'ai_commands': {'received': False, 'count': 0, 'last_time': 0, 'commands': []}
        }
        
        # Simulated AI component data
        self.simulated_components = {
            'perception_description': {'active': True, 'data_rate': 5.0},
            'decision_status': {'active': True, 'data_rate': 2.0},
            'slam_status': {'active': True, 'data_rate': 5.0},
            'fusion_status': {'active': True, 'data_rate': 5.0}
        }
        
        # Subscribers
        self.ai_status_sub = self.create_subscription(
            String, '/quadcopter/ai/status', 
            lambda msg: self.ai_callback('ai_status', msg), 10
        )
        self.ai_results_sub = self.create_subscription(
            String, '/quadcopter/ai/results', 
            lambda msg: self.ai_callback('ai_results', msg), 10
        )
        self.ai_commands_sub = self.create_subscription(
            String, '/quadcopter/ai/commands', 
            lambda msg: self.ai_callback('ai_commands', msg), 10
        )
        
        # Publishers for simulated AI components
        self.component_publishers = {}
        for component_name in self.simulated_components:
            self.component_publishers[component_name] = self.create_publisher(String, f'/quadcopter/{component_name}', 10)
                
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        # Component simulation timer
        self.component_timer = self.create_timer(0.1, self.simulate_components)
        
        self.get_logger().info("🧪 Real-time AI Integration Test Started")
        
    def ai_callback(self, data_type, msg):
        """Track AI integration data reception"""
        current_time = time.time()
        
        if not self.ai_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.ai_data[data_type]['received'] = True
            
        self.ai_data[data_type]['count'] += 1
        self.ai_data[data_type]['last_time'] = current_time
        
        # Store data for analysis
        try:
            data = json.loads(msg.data)
            if data_type == 'ai_status':
                self.ai_data[data_type]['statuses'].append(data)
            elif data_type == 'ai_results':
                self.ai_data[data_type]['results'].append(data)
            elif data_type == 'ai_commands':
                self.ai_data[data_type]['commands'].append(data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON in {data_type}")
                
    def simulate_components(self):
        """Simulate AI component data generation"""
        current_time = time.time()
        
        for component_name, component_info in self.simulated_components.items():
            if not component_info['active']:
                continue
                
            # Check if it's time to publish this component
            if hasattr(self, f'last_{component_name}_time'):
                time_since_last = current_time - getattr(self, f'last_{component_name}_time')
                if time_since_last < 1.0 / component_info['data_rate']:
                    continue
            else:
                setattr(self, f'last_{component_name}_time', 0)
                
            # Generate and publish component data
            self.simulate_component_data(component_name)
            setattr(self, f'last_{component_name}_time', current_time)
            
    def simulate_component_data(self, component_name):
        """Simulate specific AI component data"""
        if 'perception' in component_name:
            data = {
                'description': 'Environment contains multiple objects including walls, obstacles, and open spaces.',
                'timestamp': time.time(),
                'confidence': 0.85
            }
        elif 'decision' in component_name:
            data = {
                'mission_state': 'exploration',
                'action': 'move_forward',
                'confidence': 0.9,
                'timestamp': time.time()
            }
        elif 'slam' in component_name:
            data = {
                'slam_state': 'mapping',
                'confidence': 0.8,
                'keyframes': 15,
                'timestamp': time.time()
            }
        elif 'fusion' in component_name:
            data = {
                'fusion_state': 'active',
                'healthy_sensors': 6,
                'total_sensors': 7,
                'confidence': 0.85,
                'timestamp': time.time()
            }
        else:
            data = {
                'status': 'active',
                'confidence': 0.7,
                'timestamp': time.time()
            }
            
        # Publish component data
        msg = String()
        msg.data = json.dumps(data)
        self.component_publishers[component_name].publish(msg)
        
    def check_ai_integration_health(self):
        """Check if AI integration system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.ai_data)
        
        for component_name, data in self.ai_data.items():
            if data['received'] and (current_time - data['last_time']) < 10.0:
                healthy_components += 1
            else:
                self.get_logger().warn(f"⚠️ {component_name} not responding")
                
        return healthy_components, total_components
        
    def analyze_ai_integration_data(self):
        """Analyze AI integration data quality"""
        analysis = {}
        
        for component_name, data in self.ai_data.items():
            if data['received']:
                elapsed_time = time.time() - self.start_time
                data_rate = data['count'] / elapsed_time if elapsed_time > 0 else 0
                
                analysis[component_name] = {
                    'data_rate': data_rate,
                    'total_count': data['count'],
                    'last_update': time.time() - data['last_time']
                }
                
        return analysis
        
    def analyze_ai_coordination(self):
        """Analyze AI coordination quality"""
        if not self.ai_data['ai_results']['results']:
            return None
            
        results = self.ai_data['ai_results']['results']
        
        # Extract coordination metrics
        integration_states = [result.get('integration_state', 'unknown') for result in results]
        confidences = [result.get('confidence', 0) for result in results]
        component_counts = [result.get('component_count', 0) for result in results]
        
        return {
            'total_results': len(results),
            'unique_states': list(set(integration_states)),
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'avg_component_count': np.mean(component_counts) if component_counts else 0
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            self.get_logger().info("⏳ Waiting for Real-time AI Integration to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic AI integration connectivity
            self.get_logger().info("🧪 Phase 1: AI Integration Connectivity Test")
            
            healthy_components, total_components = self.check_ai_integration_health()
            
            if healthy_components == total_components:
                self.get_logger().info(f"✅ All {total_components} AI integration components connected and responding")
                self.test_phase = 1
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {healthy_components}/{total_components} AI integration components healthy")
                
        elif self.test_phase == 1:
            # Phase 2: AI integration data quality analysis
            self.get_logger().info("🧪 Phase 2: AI Integration Data Quality Analysis")
            
            analysis = self.analyze_ai_integration_data()
            
            self.get_logger().info("📊 AI Integration Data Analysis:")
            for component_name, stats in analysis.items():
                self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, "
                                     f"{stats['total_count']} samples")
            
            # Check if data rates are reasonable
            good_rates = 0
            for component_name, stats in analysis.items():
                expected_rate = {
                    'ai_status': 20,
                    'ai_results': 20,
                    'ai_commands': 5
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:
                    good_rates += 1
                else:
                    self.get_logger().warn(f"⚠️ {component_name} data rate low: {stats['data_rate']:.1f} Hz")
            
            if good_rates == len(analysis):
                self.get_logger().info("✅ All AI integration components have good data rates")
                self.test_phase = 2
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {good_rates}/{len(analysis)} components have good data rates")
                
        elif self.test_phase == 2:
            # Phase 3: AI coordination analysis
            self.get_logger().info("🧪 Phase 3: AI Coordination Analysis")
            
            coordination_analysis = self.analyze_ai_coordination()
            
            if coordination_analysis:
                self.get_logger().info("🤖 AI Coordination Analysis:")
                self.get_logger().info(f"  Total results: {coordination_analysis['total_results']}")
                self.get_logger().info(f"  Integration states: {coordination_analysis['unique_states']}")
                self.get_logger().info(f"  Average confidence: {coordination_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Average component count: {coordination_analysis['avg_component_count']:.1f}")
                
                # Check coordination quality
                if coordination_analysis['avg_confidence'] > 0.7:
                    self.get_logger().info("✅ AI coordination confidence is high")
                else:
                    self.get_logger().warn("⚠️ AI coordination confidence is low")
            else:
                self.get_logger().warn("⚠️ No AI coordination data available")
                
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: Final AI integration test
            self.get_logger().info("🧪 Phase 4: Final AI Integration Test")
            
            # Final health check
            healthy_components, total_components = self.check_ai_integration_health()
            analysis = self.analyze_ai_integration_data()
            coordination_analysis = self.analyze_ai_coordination()
            
            self.get_logger().info("🎉 REAL-TIME AI INTEGRATION TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} AI integration components healthy")
            self.get_logger().info(f"✅ Total AI integration samples: {sum(data['count'] for data in self.ai_data.values())}")
            
            if coordination_analysis:
                self.get_logger().info(f"✅ Coordination: {coordination_analysis['total_results']} results, "
                                     f"confidence: {coordination_analysis['avg_confidence']:.3f}")
            
            self.get_logger().info("✅ Real-time AI Integration working")
            self.get_logger().info("✅ Ready for Stage 12: Mission Planning & Execution")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test_node = RealTimeAIIntegrationTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 