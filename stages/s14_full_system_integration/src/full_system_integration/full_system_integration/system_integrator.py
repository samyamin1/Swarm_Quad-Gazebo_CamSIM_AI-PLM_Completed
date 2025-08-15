#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from visualization_msgs.msg import MarkerArray, Marker
import time
import json
import numpy as np
from typing import Dict, List, Optional
import threading

class SystemComponent:
    def __init__(self, name: str, status: str = "unknown"):
        self.name = name
        self.status = status
        self.last_heartbeat = time.time()
        self.performance_metrics = {}
        self.error_count = 0
        self.start_time = time.time()

class FullSystemIntegrator(Node):
    def __init__(self):
        super().__init__('full_system_integrator')
        
        # System integration parameters
        self.integration_frequency = 5.0  # Hz
        self.heartbeat_timeout = 10.0  # seconds
        self.system_startup_timeout = 30.0  # seconds
        
        # System components tracking
        self.system_components = {
            'quadcopter_simulation': SystemComponent('Quadcopter Simulation'),
            '3d_map_creation': SystemComponent('3D Map Creation'),
            'sensor_simulation': SystemComponent('Sensor Simulation'),
            'slam_engine': SystemComponent('SLAM Engine'),
            'perception_plm': SystemComponent('Perception PLM'),
            'llm_decision_engine': SystemComponent('LLM Decision Engine'),
            'navigation_control': SystemComponent('Navigation Control'),
            'swarm_coordination': SystemComponent('Swarm Coordination'),
            'multi_sensor_fusion': SystemComponent('Multi-Sensor Fusion'),
            'advanced_slam': SystemComponent('Advanced SLAM'),
            'realtime_ai_integration': SystemComponent('Real-time AI Integration'),
            'mission_planning': SystemComponent('Mission Planning'),
            'performance_optimization': SystemComponent('Performance Optimization'),
            'full_system_integration': SystemComponent('Full System Integration')
        }
        
        # System state
        self.system_state = "initializing"
        self.startup_time = time.time()
        self.operational_components = 0
        self.total_components = len(self.system_components)
        
        # Publishers
        self.system_status_pub = self.create_publisher(String, '/quadcopter/system/status', 10)
        self.system_health_pub = self.create_publisher(String, '/quadcopter/system/health', 10)
        self.system_commands_pub = self.create_publisher(String, '/quadcopter/system/commands', 10)
        self.system_visualization_pub = self.create_publisher(MarkerArray, '/quadcopter/system/visualization', 10)
        
        # Subscribers for component status
        self.setup_component_subscribers()
        
        # System integration timer
        self.integration_timer = self.create_timer(1.0/self.integration_frequency, self.system_integration_loop)
        
        # System health monitoring timer
        self.health_timer = self.create_timer(2.0, self.system_health_monitoring)
        
        self.get_logger().info("Full System Integrator initialized")
        
    def setup_component_subscribers(self):
        """Setup subscribers for all system components"""
        # Stage 1: Quadcopter Simulation
        self.quadcopter_status_sub = self.create_subscription(
            String, '/quadcopter/status', 
            lambda msg: self.component_status_callback('quadcopter_simulation', msg), 10
        )
        
        # Stage 2: 3D Map Creation
        self.map_status_sub = self.create_subscription(
            String, '/quadcopter/map/status', 
            lambda msg: self.component_status_callback('3d_map_creation', msg), 10
        )
        
        # Stage 3: Sensor Simulation
        self.sensor_status_sub = self.create_subscription(
            String, '/quadcopter/sensor/status', 
            lambda msg: self.component_status_callback('sensor_simulation', msg), 10
        )
        
        # Stage 4: SLAM Engine
        self.slam_status_sub = self.create_subscription(
            String, '/quadcopter/slam/status', 
            lambda msg: self.component_status_callback('slam_engine', msg), 10
        )
        
        # Stage 5: Perception PLM
        self.perception_status_sub = self.create_subscription(
            String, '/quadcopter/perception/status', 
            lambda msg: self.component_status_callback('perception_plm', msg), 10
        )
        
        # Stage 6: LLM Decision Engine
        self.decision_status_sub = self.create_subscription(
            String, '/quadcopter/decision/status', 
            lambda msg: self.component_status_callback('llm_decision_engine', msg), 10
        )
        
        # Stage 7: Navigation Control
        self.navigation_status_sub = self.create_subscription(
            String, '/quadcopter/navigation/status', 
            lambda msg: self.component_status_callback('navigation_control', msg), 10
        )
        
        # Stage 8: Swarm Coordination
        self.swarm_status_sub = self.create_subscription(
            String, '/quadcopter/swarm/status', 
            lambda msg: self.component_status_callback('swarm_coordination', msg), 10
        )
        
        # Stage 9: Multi-Sensor Fusion
        self.fusion_status_sub = self.create_subscription(
            String, '/quadcopter/fusion/status', 
            lambda msg: self.component_status_callback('multi_sensor_fusion', msg), 10
        )
        
        # Stage 10: Advanced SLAM
        self.advanced_slam_status_sub = self.create_subscription(
            String, '/quadcopter/advanced_slam/status', 
            lambda msg: self.component_status_callback('advanced_slam', msg), 10
        )
        
        # Stage 11: Real-time AI Integration
        self.ai_integration_status_sub = self.create_subscription(
            String, '/quadcopter/ai/status', 
            lambda msg: self.component_status_callback('realtime_ai_integration', msg), 10
        )
        
        # Stage 12: Mission Planning
        self.mission_status_sub = self.create_subscription(
            String, '/quadcopter/mission/status', 
            lambda msg: self.component_status_callback('mission_planning', msg), 10
        )
        
        # Stage 13: Performance Optimization
        self.performance_status_sub = self.create_subscription(
            String, '/quadcopter/performance/status', 
            lambda msg: self.component_status_callback('performance_optimization', msg), 10
        )
        
    def component_status_callback(self, component_name: str, msg):
        """Handle component status updates"""
        if component_name in self.system_components:
            component = self.system_components[component_name]
            component.last_heartbeat = time.time()
            
            try:
                status_data = json.loads(msg.data)
                component.status = status_data.get('status', 'active')
                component.performance_metrics = status_data
            except json.JSONDecodeError:
                component.status = 'active'  # Assume active if JSON parsing fails
                
    def check_system_startup(self):
        """Check if all system components have started"""
        current_time = time.time()
        elapsed_time = current_time - self.startup_time
        
        # Check if startup timeout exceeded
        if elapsed_time > self.system_startup_timeout:
            self.system_state = "operational"
            return True
            
        # Check if all components are responding
        responding_components = 0
        for component in self.system_components.values():
            if current_time - component.last_heartbeat < self.heartbeat_timeout:
                responding_components += 1
                
        if responding_components == self.total_components:
            self.system_state = "operational"
            return True
            
        return False
        
    def check_component_health(self):
        """Check health of all system components"""
        current_time = time.time()
        healthy_components = 0
        
        for component in self.system_components.values():
            time_since_heartbeat = current_time - component.last_heartbeat
            
            if time_since_heartbeat < self.heartbeat_timeout:
                if component.status in ['active', 'ready', 'operational']:
                    component.status = 'healthy'
                    healthy_components += 1
                else:
                    component.status = 'warning'
            else:
                component.status = 'offline'
                component.error_count += 1
                
        self.operational_components = healthy_components
        
        # Update system state based on component health
        if healthy_components == self.total_components:
            self.system_state = "fully_operational"
        elif healthy_components > self.total_components * 0.8:
            self.system_state = "operational"
        elif healthy_components > self.total_components * 0.5:
            self.system_state = "degraded"
        else:
            self.system_state = "critical"
            
    def system_integration_loop(self):
        """Main system integration loop"""
        current_time = time.time()
        
        # Check system startup
        if self.system_state == "initializing":
            if self.check_system_startup():
                self.get_logger().info("System startup completed")
                
        # Check component health
        self.check_component_health()
        
        # Publish system status
        self.publish_system_status()
        
        # Publish system visualization
        self.publish_system_visualization()
        
        # Log system status
        if current_time % 10 < 1:  # Log every 10 seconds
            self.get_logger().info(f"System State: {self.system_state}, "
                                  f"Components: {self.operational_components}/{self.total_components}")
                                  
    def system_health_monitoring(self):
        """Monitor system health and trigger recovery actions"""
        # Check for critical system issues
        if self.system_state == "critical":
            self.trigger_system_recovery()
            
        # Check for degraded performance
        elif self.system_state == "degraded":
            self.trigger_performance_optimization()
            
    def trigger_system_recovery(self):
        """Trigger system recovery procedures"""
        self.get_logger().warn("CRITICAL: Triggering system recovery")
        
        recovery_commands = {
            'timestamp': time.time(),
            'type': 'system_recovery',
            'priority': 'critical',
            'actions': [
                'restart_failed_components',
                'clear_error_states',
                'reinitialize_critical_systems'
            ]
        }
        
        commands_msg = String()
        commands_msg.data = json.dumps(recovery_commands)
        self.system_commands_pub.publish(commands_msg)
        
    def trigger_performance_optimization(self):
        """Trigger performance optimization"""
        self.get_logger().info("DEGRADED: Triggering performance optimization")
        
        optimization_commands = {
            'timestamp': time.time(),
            'type': 'performance_optimization',
            'priority': 'medium',
            'actions': [
                'memory_cleanup',
                'computation_optimization',
                'network_optimization'
            ]
        }
        
        commands_msg = String()
        commands_msg.data = json.dumps(optimization_commands)
        self.system_commands_pub.publish(commands_msg)
        
    def publish_system_status(self):
        """Publish complete system status"""
        status_data = {
            'timestamp': time.time(),
            'system_state': self.system_state,
            'operational_components': self.operational_components,
            'total_components': self.total_components,
            'startup_time': self.startup_time,
            'uptime': time.time() - self.startup_time,
            'components': {}
        }
        
        # Add component details
        for component_name, component in self.system_components.items():
            status_data['components'][component_name] = {
                'status': component.status,
                'last_heartbeat': component.last_heartbeat,
                'error_count': component.error_count,
                'uptime': time.time() - component.start_time
            }
            
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.system_status_pub.publish(status_msg)
        
    def publish_system_health(self):
        """Publish system health diagnostics"""
        health_data = {
            'timestamp': time.time(),
            'system_health': self.calculate_system_health(),
            'component_health': {},
            'recommendations': self.generate_health_recommendations()
        }
        
        # Add component health details
        for component_name, component in self.system_components.items():
            health_data['component_health'][component_name] = {
                'health_score': self.calculate_component_health(component),
                'status': component.status,
                'error_rate': component.error_count / max(1, (time.time() - component.start_time) / 3600)
            }
            
        health_msg = String()
        health_msg.data = json.dumps(health_data)
        self.system_health_pub.publish(health_msg)
        
    def calculate_system_health(self) -> float:
        """Calculate overall system health score (0.0 to 1.0)"""
        if self.operational_components == 0:
            return 0.0
            
        # Base health from operational components
        base_health = self.operational_components / self.total_components
        
        # Penalty for error counts
        total_errors = sum(component.error_count for component in self.system_components.values())
        error_penalty = min(0.3, total_errors * 0.05)  # Max 30% penalty
        
        # Time-based health (system gets more stable over time)
        uptime_hours = (time.time() - self.startup_time) / 3600
        stability_bonus = min(0.1, uptime_hours * 0.01)  # Max 10% bonus
        
        health_score = base_health - error_penalty + stability_bonus
        return max(0.0, min(1.0, health_score))
        
    def calculate_component_health(self, component: SystemComponent) -> float:
        """Calculate individual component health score"""
        if component.status == 'healthy':
            base_score = 1.0
        elif component.status == 'warning':
            base_score = 0.7
        elif component.status == 'offline':
            base_score = 0.0
        else:
            base_score = 0.5
            
        # Penalty for errors
        error_penalty = min(0.3, component.error_count * 0.1)
        
        # Bonus for uptime
        uptime_hours = (time.time() - component.start_time) / 3600
        stability_bonus = min(0.1, uptime_hours * 0.01)
        
        health_score = base_score - error_penalty + stability_bonus
        return max(0.0, min(1.0, health_score))
        
    def generate_health_recommendations(self) -> List[str]:
        """Generate health improvement recommendations"""
        recommendations = []
        
        # Check for offline components
        offline_components = [name for name, comp in self.system_components.items() 
                            if comp.status == 'offline']
        if offline_components:
            recommendations.append(f"Restart offline components: {', '.join(offline_components)}")
            
        # Check for high error rates
        high_error_components = [name for name, comp in self.system_components.items() 
                               if comp.error_count > 5]
        if high_error_components:
            recommendations.append(f"Investigate high error rates in: {', '.join(high_error_components)}")
            
        # Check for performance issues
        if self.system_state == "degraded":
            recommendations.append("Consider performance optimization")
            
        # Check for startup issues
        if time.time() - self.startup_time < 60:  # First minute
            recommendations.append("System still starting up, monitor component initialization")
            
        if not recommendations:
            recommendations.append("System health is good, continue monitoring")
            
        return recommendations
        
    def publish_system_visualization(self):
        """Publish system visualization markers"""
        marker_array = MarkerArray()
        
        # Component status visualization
        for i, (component_name, component) in enumerate(self.system_components.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "system_components"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = (i % 5) * 3.0
            marker.pose.position.y = (i // 5) * 3.0
            marker.pose.position.z = 0.0
            
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.8
            
            # Color based on component status
            if component.status == 'healthy':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif component.status == 'warning':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif component.status == 'offline':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
        # System health indicator
        health_marker = Marker()
        health_marker.header.frame_id = "map"
        health_marker.header.stamp = self.get_clock().now().to_msg()
        health_marker.ns = "system_health"
        health_marker.id = 100
        health_marker.type = Marker.CYLINDER
        health_marker.action = Marker.ADD
        
        health_score = self.calculate_system_health()
        health_marker.pose.position.x = 0.0
        health_marker.pose.position.y = 10.0
        health_marker.pose.position.z = health_score * 3.0
        
        health_marker.scale.x = 2.0
        health_marker.scale.y = 2.0
        health_marker.scale.z = 3.0
        
        # Color based on system health
        if health_score > 0.8:
            health_marker.color.r = 0.0
            health_marker.color.g = 1.0
            health_marker.color.b = 0.0
        elif health_score > 0.6:
            health_marker.color.r = 1.0
            health_marker.color.g = 1.0
            health_marker.color.b = 0.0
        else:
            health_marker.color.r = 1.0
            health_marker.color.g = 0.0
            health_marker.color.b = 0.0
            
        health_marker.color.a = 0.8
        marker_array.markers.append(health_marker)
        
        self.system_visualization_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    system_integrator = FullSystemIntegrator()
    
    try:
        rclpy.spin(system_integrator)
    except KeyboardInterrupt:
        system_integrator.get_logger().info("Full System Integrator stopped by user")
    finally:
        system_integrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 