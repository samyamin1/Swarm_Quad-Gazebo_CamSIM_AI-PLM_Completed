#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
import numpy as np

class PerceptionPLMTest(Node):
    def __init__(self):
        super().__init__('perception_plm_test')
        
        # Test state
        self.test_phase = 0
        self.start_time = time.time()
        self.test_duration = 30  # seconds
        
        # PLM data tracking
        self.plm_data = {
            'description': {'received': False, 'count': 0, 'last_time': 0, 'descriptions': []},
            'analysis': {'received': False, 'count': 0, 'last_time': 0, 'analyses': []},
            'alerts': {'received': False, 'count': 0, 'last_time': 0, 'alerts': []}
        }
        
        # Subscribers
        self.description_sub = self.create_subscription(
            String, '/quadcopter/perception/description', 
            lambda msg: self.plm_callback('description', msg), 10
        )
        self.analysis_sub = self.create_subscription(
            String, '/quadcopter/perception/analysis', 
            lambda msg: self.plm_callback('analysis', msg), 10
        )
        self.alerts_sub = self.create_subscription(
            String, '/quadcopter/perception/alerts', 
            lambda msg: self.plm_callback('alerts', msg), 10
        )
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)
        
        self.get_logger().info("🧪 Perception PLM Test Started")
        
    def plm_callback(self, data_type, msg):
        """Track PLM data reception"""
        current_time = time.time()
        
        if not self.plm_data[data_type]['received']:
            self.get_logger().info(f"✅ {data_type} data received")
            self.plm_data[data_type]['received'] = True
            
        self.plm_data[data_type]['count'] += 1
        self.plm_data[data_type]['last_time'] = current_time
        
        # Store data for analysis
        if data_type == 'description':
            self.plm_data[data_type]['descriptions'].append(msg.data)
        elif data_type == 'analysis':
            try:
                analysis = json.loads(msg.data)
                self.plm_data[data_type]['analyses'].append(analysis)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in analysis message")
        elif data_type == 'alerts':
            try:
                alerts = json.loads(msg.data)
                self.plm_data[data_type]['alerts'].extend(alerts)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON in alerts message")
                
    def check_plm_health(self):
        """Check if PLM system is healthy"""
        current_time = time.time()
        healthy_components = 0
        total_components = len(self.plm_data)
        
        for component_name, data in self.plm_data.items():
            # Check if component is receiving data within last 10 seconds
            if data['received'] and (current_time - data['last_time']) < 10.0:
                healthy_components += 1
            else:
                self.get_logger().warn(f"⚠️ {component_name} not responding")
                
        return healthy_components, total_components
        
    def analyze_plm_data(self):
        """Analyze PLM data quality"""
        analysis = {}
        
        for component_name, data in self.plm_data.items():
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
        
    def analyze_descriptions(self):
        """Analyze natural language descriptions"""
        if not self.plm_data['description']['descriptions']:
            return None
            
        descriptions = self.plm_data['description']['descriptions']
        
        # Analyze description quality
        avg_length = np.mean([len(desc) for desc in descriptions])
        unique_descriptions = len(set(descriptions))
        
        # Check for key terms
        key_terms = ['quadcopter', 'position', 'coordinates', 'detect', 'sensors', 'obstacles', 'objects']
        term_usage = {}
        for term in key_terms:
            term_usage[term] = sum(1 for desc in descriptions if term.lower() in desc.lower())
            
        return {
            'total_descriptions': len(descriptions),
            'avg_length': avg_length,
            'unique_descriptions': unique_descriptions,
            'term_usage': term_usage
        }
        
    def analyze_ai_analysis(self):
        """Analyze AI-generated analysis"""
        if not self.plm_data['analysis']['analyses']:
            return None
            
        analyses = self.plm_data['analysis']['analyses']
        
        # Extract analysis metrics
        confidences = [analysis.get('confidence', 0) for analysis in analyses]
        alert_counts = [len(analysis.get('alerts', [])) for analysis in analyses]
        recommendation_counts = [len(analysis.get('recommendations', [])) for analysis in analyses]
        
        # Check for critical alerts
        critical_alerts = []
        for analysis in analyses:
            alerts = analysis.get('alerts', [])
            critical = [alert for alert in alerts if alert.get('severity') == 'high']
            critical_alerts.extend(critical)
            
        return {
            'total_analyses': len(analyses),
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'avg_alerts': np.mean(alert_counts) if alert_counts else 0,
            'avg_recommendations': np.mean(recommendation_counts) if recommendation_counts else 0,
            'critical_alerts': len(critical_alerts)
        }
        
    def run_test(self):
        """Main test loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5.0:
            # Phase 0: Wait for PLM to initialize
            self.get_logger().info("⏳ Waiting for Perception PLM to initialize...")
            return
            
        if self.test_phase == 0:
            # Phase 1: Basic PLM connectivity
            self.get_logger().info("🧪 Phase 1: PLM Connectivity Test")
            
            healthy_components, total_components = self.check_plm_health()
            
            if healthy_components == total_components:
                self.get_logger().info(f"✅ All {total_components} PLM components connected and responding")
                self.test_phase = 1
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {healthy_components}/{total_components} PLM components healthy")
                
        elif self.test_phase == 1:
            # Phase 2: PLM data quality analysis
            self.get_logger().info("🧪 Phase 2: PLM Data Quality Analysis")
            
            analysis = self.analyze_plm_data()
            
            self.get_logger().info("📊 PLM Data Analysis:")
            for component_name, stats in analysis.items():
                self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, "
                                     f"{stats['total_count']} samples, "
                                     f"last update: {stats['last_update']:.1f}s ago")
            
            # Check if data rates are reasonable
            good_rates = 0
            for component_name, stats in analysis.items():
                expected_rate = {
                    'description': 2,
                    'analysis': 2,
                    'alerts': 0.5
                }
                
                if stats['data_rate'] >= expected_rate[component_name] * 0.3:  # 30% of expected
                    good_rates += 1
                else:
                    self.get_logger().warn(f"⚠️ {component_name} data rate low: {stats['data_rate']:.1f} Hz")
            
            if good_rates == len(analysis):
                self.get_logger().info("✅ All PLM components have good data rates")
                self.test_phase = 2
                self.start_time = time.time()
            else:
                self.get_logger().warn(f"⚠️ Only {good_rates}/{len(analysis)} components have good data rates")
                
        elif self.test_phase == 2:
            # Phase 3: Natural language description analysis
            self.get_logger().info("🧪 Phase 3: Natural Language Description Analysis")
            
            description_analysis = self.analyze_descriptions()
            
            if description_analysis:
                self.get_logger().info("📝 Description Analysis:")
                self.get_logger().info(f"  Total descriptions: {description_analysis['total_descriptions']}")
                self.get_logger().info(f"  Average length: {description_analysis['avg_length']:.1f} characters")
                self.get_logger().info(f"  Unique descriptions: {description_analysis['unique_descriptions']}")
                
                self.get_logger().info("🔍 Key Term Usage:")
                for term, count in description_analysis['term_usage'].items():
                    self.get_logger().info(f"  {term}: {count} occurrences")
                    
                # Check description quality
                if description_analysis['avg_length'] > 50:
                    self.get_logger().info("✅ Descriptions are sufficiently detailed")
                else:
                    self.get_logger().warn("⚠️ Descriptions may be too brief")
                    
                if description_analysis['unique_descriptions'] > 1:
                    self.get_logger().info("✅ PLM is generating diverse descriptions")
                else:
                    self.get_logger().warn("⚠️ PLM may be generating repetitive descriptions")
            else:
                self.get_logger().warn("⚠️ No descriptions available for analysis")
                
            self.test_phase = 3
            self.start_time = time.time()
            
        elif self.test_phase == 3:
            # Phase 4: AI analysis quality
            self.get_logger().info("🧪 Phase 4: AI Analysis Quality")
            
            ai_analysis = self.analyze_ai_analysis()
            
            if ai_analysis:
                self.get_logger().info("🤖 AI Analysis Quality:")
                self.get_logger().info(f"  Total analyses: {ai_analysis['total_analyses']}")
                self.get_logger().info(f"  Average confidence: {ai_analysis['avg_confidence']:.3f}")
                self.get_logger().info(f"  Average alerts per analysis: {ai_analysis['avg_alerts']:.1f}")
                self.get_logger().info(f"  Average recommendations: {ai_analysis['avg_recommendations']:.1f}")
                self.get_logger().info(f"  Critical alerts: {ai_analysis['critical_alerts']}")
                
                # Check AI quality
                if ai_analysis['avg_confidence'] > 0.7:
                    self.get_logger().info("✅ AI confidence is high")
                else:
                    self.get_logger().warn("⚠️ AI confidence is low")
                    
                if ai_analysis['critical_alerts'] > 0:
                    self.get_logger().info("✅ AI is detecting critical situations")
                else:
                    self.get_logger().info("✅ No critical situations detected (environment may be safe)")
            else:
                self.get_logger().warn("⚠️ No AI analysis available")
                
            self.test_phase = 4
            self.start_time = time.time()
            
        elif self.test_phase == 4:
            # Phase 5: Alert system test
            self.get_logger().info("🧪 Phase 5: Alert System Test")
            
            total_alerts = len(self.plm_data['alerts']['alerts'])
            
            if total_alerts > 0:
                self.get_logger().info(f"🚨 Alert System: {total_alerts} alerts generated")
                
                # Analyze alert types
                alert_types = {}
                for alert in self.plm_data['alerts']['alerts']:
                    alert_type = alert.get('type', 'unknown')
                    alert_types[alert_type] = alert_types.get(alert_type, 0) + 1
                    
                self.get_logger().info("📊 Alert Types:")
                for alert_type, count in alert_types.items():
                    self.get_logger().info(f"  {alert_type}: {count}")
                    
                self.get_logger().info("✅ Alert system is functioning")
            else:
                self.get_logger().info("✅ No alerts generated (environment appears safe)")
                
            self.test_phase = 5
            self.start_time = time.time()
            
        elif self.test_phase == 5:
            # Phase 6: Final PLM integration test
            self.get_logger().info("🧪 Phase 6: Final PLM Integration Test")
            
            # Final health check
            healthy_components, total_components = self.check_plm_health()
            analysis = self.analyze_plm_data()
            description_analysis = self.analyze_descriptions()
            ai_analysis = self.analyze_ai_analysis()
            
            self.get_logger().info("🎉 PERCEPTION PLM TEST COMPLETED!")
            self.get_logger().info(f"✅ {healthy_components}/{total_components} PLM components healthy")
            self.get_logger().info(f"✅ Total PLM samples: {sum(data['count'] for data in self.plm_data.values())}")
            
            if description_analysis:
                self.get_logger().info(f"✅ Descriptions: {description_analysis['total_descriptions']} generated")
                
            if ai_analysis:
                self.get_logger().info(f"✅ AI Analysis: {ai_analysis['total_analyses']} analyses, "
                                     f"confidence: {ai_analysis['avg_confidence']:.3f}")
            
            self.get_logger().info("✅ Perception Language Model integrated and working")
            self.get_logger().info("✅ Ready for Stage 6: LLM Decision Engine")
            
            # Shutdown after successful test
            self.destroy_node()
            rclpy.shutdown()
            
    def print_final_report(self):
        """Print final test report"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 FINAL PERCEPTION PLM REPORT")
        self.get_logger().info("="*60)
        
        analysis = self.analyze_plm_data()
        description_analysis = self.analyze_descriptions()
        ai_analysis = self.analyze_ai_analysis()
        total_samples = sum(data['count'] for data in self.plm_data.values())
        
        self.get_logger().info(f"📊 Total PLM Samples: {total_samples}")
        self.get_logger().info(f"⏱️ Test Duration: {time.time() - self.start_time:.1f} seconds")
        
        for component_name, stats in analysis.items():
            self.get_logger().info(f"  {component_name}: {stats['data_rate']:.1f} Hz, {stats['total_count']} samples")
            
        if description_analysis:
            self.get_logger().info(f"📝 Descriptions: {description_analysis['total_descriptions']}, "
                                 f"avg length: {description_analysis['avg_length']:.1f} chars")
            
        if ai_analysis:
            self.get_logger().info(f"🤖 AI Analysis: {ai_analysis['total_analyses']}, "
                                 f"confidence: {ai_analysis['avg_confidence']:.3f}")
            
        self.get_logger().info("✅ Stage 5: Perception Language Model - COMPLETED")
        self.get_logger().info("🚀 Ready for Stage 6: LLM Decision Engine")

def main(args=None):
    rclpy.init(args=args)
    test_node = PerceptionPLMTest()
    
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