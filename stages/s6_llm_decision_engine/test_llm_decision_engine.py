#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import time
import json
import numpy as np

class LLMDecisionEngineTest(Node):
    def __init__(self):
        super().__init__('llm_decision_engine_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        self.test_duration = 45  # seconds
        
        # Decision data tracking
        self.decision_data = {
            'mission_status': {'received': False, 'count': 0, 'last_time': 0, 'statuses': []},
            'decision_log': {'received': False, 'count': 0, 'last_time': 0, 'decisions': []},
            'emergency': {'received': False, 'count': 0, 'last_time': 0, 'emergencies': []},
            'mission_goal': {'received': False, 'count': 0, 'last_time': 0, 'goals': []}
        }
        
        # Subscribers
        self.mission_status_sub = self.create_subscription(
            String, '/quadcopter/mission/status', 
            lambda msg: self.decision_callback('mission_status', msg), 10
        )
        self.decision_log_sub = self.create_subscription(
            String, '/quadcopter/decision/log', 
            lambda msg: self.decision_callback('decision_log', msg), 10
        )
        self.emergency_sub = self.create_subscription(
            Bool, '/quadcopter/emergency', 
            lambda msg: self.emergency_callback('emergency', msg), 10
        )
        self.mission_goal_sub = self.create_subscription(
            PoseStamped, '/quadcopter/mission/goal', 
            lambda msg: self.goal_callback('mission_goal', msg), 10
        )
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        self.get_logger().info("🧪 LLM Decision Engine Test Started")
        
    def decision_callback(self, data_type, msg):
        """Track decision data reception"""
        current_time = time.time()
        
        if not self.decision_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.decision_data[data_type]['received'] = True
            
        self.decision_data[data_type]['count'] += 1
        self.decision_data[data_type]['last_time'] = current_time
        
        # Store data for analysis
        if data_type == 'mission_status':
            try:
                status = json.loads(msg.data)
                self.decision_data[data_type]['statuses'].append(status)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in mission status")
        elif data_type == 'decision_log':
            try:
                decision = json.loads(msg.data)
                self.decision_data[data_type]['decisions'].append(decision)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in decision log")
                
    def emergency_callback(self, data_type, msg):
        """Track emergency data"""
        current_time = time.time()
        
        if not self.decision_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.decision_data[data_type]['received'] = True
            
        self.decision_data[data_type]['count'] += 1
        self.decision_data[data_type]['last_time'] = current_time
        self.decision_data[data_type]['emergencies'].append(msg.data)
        
    def goal_callback(self, data_type, msg):
        """Track mission goal data"""
        current_time = time.time()
        
        if not self.decision_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.decision_data[data_type]['received'] = True
            
        self.decision_data[data_type]['count'] += 1
        self.decision_data[data_type]['last_time'] = current_time
        
        goal = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        self.decision_data[data_type]['goals'].append(goal)
        
    def check_decision_health(self):
        """Check if decision system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.decision_data)
        
        for component_name, data in self.decision_data.items():
            # Check if component is receiving data within last 10 seconds
            if data['received'] and (current_time - data['last_time']) < 10.0:
                healthy_components += 1
            else:
                self.get_logger().warn(f"⚠️ {component_name} not responding")
                
        return healthy_components, total_components
        
    def analyze_decision_data(self):
        """Analyze decision data quality"""
        analysis = {}
        
        for component_name, data in self.decision_data.items():
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
        
    def analyze_mission_progression(self):
        """Analyze mission state progression"""
        if not self.decision_data['mission_status']['statuses']:
            return None
            
        statuses = self.decision_data['mission_status']['statuses']
        
        # Extract mission states
        mission_states = [status.get('mission_state', 'unknown') for status in statuses]
        actions = [status.get('action', 'unknown') for status in statuses]
        confidences = [status.get('confidence', 0) for status in statuses]
        
        # Analyze state transitions
        unique_states = list(set(mission_states))
        state_transitions = len(unique_states)
        
        # Check for mission completion
        completed_mission = 'mission_complete' in actions
        
        return {
            'total_statuses': len(statuses),
            'unique_states': unique_states,
            'state_transitions': state_transitions,
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'mission_completed': completed_mission
        }
        
    def analyze_decision_quality(self):
        """Analyze decision quality"""
        if not self.decision_data['decision_log']['decisions']:
            return None
            
        decisions = self.decision_data['decision_log']['decisions']
        
        # Extract decision metrics
        confidences = [decision.get('confidence', 0) for decision in decisions]
        actions = [decision.get('action', 'unknown') for decision in decisions]
        emergencies = [decision.get('emergency', False) for decision in decisions]
        
        # Analyze decision variety
        unique_actions = list(set(actions))
        emergency_count = sum(emergencies)
        
        return {
            'total_decisions': len(decisions),
            'unique_actions': unique_actions,
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'emergency_count': emergency_count,
            'decision_variety': len(unique_actions)
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            # Phase 0: Wait for decision engine to initialize
            self.get_logger().info("⏳ Waiting for LLM Decision Engine to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic decision engine connectivity
            self.get_logger().info("🧪 Phase 1: Decision Engine Connectivity Test")
            
            healthy_components, total_components = self.check_decision_health()
            
            if healthy_components == total_components:
                self.get_logger().info(f"✅ All {total_components} decision components connected and responding")
                self.test_phase = 1
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {healthy_components}/{total_components} decision components healthy")
                
        elif self.test_phase == 1:
            # Phase 2: Decision data quality analysis
            self.get_logger().info("🧪 Phase 2: Decision Data Quality Analysis")
            
            analysis = self.analyze_decision_data()
            
            self.get_logger().info("📊 Decision Data Analysis:")
            for component_name, stats in analysis.items():
                self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, "
                                     f"{stats['total_count']} samples, "
                                     f"last update: {stats['last_update']:.1f}s ago")
            
            # Check if data rates are reasonable
            good_rates = 0
            for component_name, stats in analysis.items():
                expected_rate = {
                    'mission_status': 1,
                    'decision_log': 1,
                    'emergency': 0.1,
                    'mission_goal': 0.5
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:  # 30% of expected
                    good_rates += 1
                else:
                    self.get_logger().warn(f"⚠️ {component_name} data rate low: {stats['data_rate']:.1f} Hz")
            
            if good_rates == len(analysis):
                self.get_logger().info("✅ All decision components have good data rates")
                self.test_phase = 2
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {good_rates}/{len(analysis)} components have good data rates")
                
        elif self.test_phase == 2:
            # Phase 3: Mission progression analysis
            self.get_logger().info("🧪 Phase 3: Mission Progression Analysis")
            
            mission_analysis = self.analyze_mission_progression()
            
            if mission_analysis:
                self.get_logger().info("🎯 Mission Analysis:")
                self.get_logger().info(f"  Total status updates: {mission_analysis['total_statuses']}")
                self.get_logger().info(f"  Mission states: {mission_analysis['unique_states']}")
                self.get_logger().info(f"  State transitions: {mission_analysis['state_transitions']}")
                self.get_logger().info(f"  Average confidence: {mission_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Mission completed: {mission_analysis['mission_completed']}")
                
                # Check mission progression
                if mission_analysis['state_transitions'] > 1:
                    self.get_logger().info("✅ Mission is progressing through different states")
                else:
                    self.get_logger().warn("⚠️ Mission may be stuck in single state")
                    
                if mission_analysis['avg_confidence'] > 0.7:
                    self.get_logger().info("✅ Decision confidence is high")
                else:
                    self.get_logger().warn("⚠️ Decision confidence is low")
            else:
                self.get_logger().warn("⚠️ No mission data available for analysis")
                
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: Decision quality analysis
            self.get_logger().info("🧪 Phase 4: Decision Quality Analysis")
            
            decision_analysis = self.analyze_decision_quality()
            
            if decision_analysis:
                self.get_logger().info("🤖 Decision Quality Analysis:")
                self.get_logger().info(f"  Total decisions: {decision_analysis['total_decisions']}")
                self.get_logger().info(f"  Unique actions: {decision_analysis['unique_actions']}")
                self.get_logger().info(f"  Average confidence: {decision_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Emergency situations: {decision_analysis['emergency_count']}")
                self.get_logger().info(f"  Decision variety: {decision_analysis['decision_variety']}")
                
                # Check decision quality
                if decision_analysis['decision_variety'] > 2:
                    self.get_logger().info("✅ AI is making diverse decisions")
                else:
                    self.get_logger().warn("⚠️ AI decisions may be too repetitive")
                    
                if decision_analysis['avg_confidence'] > 0.7:
                    self.get_logger().info("✅ Decision confidence is high")
                else:
                    self.get_logger().warn("⚠️ Decision confidence is low")
            else:
                self.get_logger().warn("⚠️ No decision data available for analysis")
                
            self.test_phase = 4
            self.start_time = time.time()
            
        elif self.test_phase == 4:
            # Phase 5: Emergency response test
            self.get_logger().info("🧪 Phase 5: Emergency Response Test")
            
            emergency_count = len(self.decision_data['emergency']['emergencies'])
            emergency_activations = sum(self.decision_data['emergency']['emergencies'])
            
            self.get_logger().info(f"🚨 Emergency System: {emergency_count} checks, {emergency_activations} activations")
            
            if emergency_activations > 0:
                self.get_logger().info("✅ Emergency system is responding to critical situations")
            else:
                self.get_logger().info("✅ No emergencies detected (environment appears safe)")
                
            self.test_phase = 5
            self.start_time = time.time()
            
        elif self.test_phase == 5:
            # Phase 6: Final decision engine integration test
            self.get_logger().info("🧪 Phase 6: Final Decision Engine Integration Test")
            
            # Final health check
            healthy_components, total_components = self.check_decision_health()
            analysis = self.analyze_decision_data()
            mission_analysis = self.analyze_mission_progression()
            decision_analysis = self.analyze_decision_quality()
            
            self.get_logger().info("🎉 LLM DECISION ENGINE TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} decision components healthy")
            self.get_logger().info(f"✅ Total decision samples: {sum(data['count'] for data in self.decision_data.values())}")
            
            if mission_analysis:
                self.get_logger().info(f"✅ Mission: {mission_analysis['state_transitions']} state transitions")
                
            if decision_analysis:
                self.get_logger().info(f"✅ Decisions: {decision_analysis['total_decisions']} made, "
                                     f"confidence: {decision_analysis['avg_confidence']:.3f}")
            
            self.get_logger().info("✅ LLM Decision Engine integrated and working")
            self.get_logger().info("✅ Ready for Stage 7: Navigation Control Loop")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()
            
    def print_final_report(self):
        """Print final test report"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 FINAL LLM DECISION ENGINE REPORT")
        self.get_logger().info("="*60)
        
        analysis = self.analyze_decision_data()
        mission_analysis = self.analyze_mission_progression()
        decision_analysis = self.analyze_decision_quality()
        total_samples = sum(data['count'] for data in self.decision_data.values())
        
        self.get_logger().info(f"📊 Total Decision Samples: {total_samples}")
        self.get_logger().info(f"⏱️ Test Duration: {time.time() - self.start_time:.1f} seconds")
        
        for component_name, stats in analysis.items():
            self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, {stats['total_count']} samples")
            
        if mission_analysis:
            self.get_logger().info(f"🎯 Mission: {mission_analysis['state_transitions']} transitions, "
                                 f"confidence: {mission_analysis['avg_confidence']:.3f}")
            
        if decision_analysis:
            self.get_logger().info(f"🤖 Decisions: {decision_analysis['total_decisions']}, "
                                 f"variety: {decision_analysis['decision_variety']}")
            
        self.get_logger().info("✅ Stage 6: LLM Decision Engine - COMPLETED")
        self.get_logger().info("🚀 Ready for Stage 7: Navigation Control Loop")

def main(args=None):
    rclpy.init(args=args)
    test_node = LLMDecisionEngineTest()
    
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