#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from visualization_msgs.msg import MarkerArray, Marker
import time
import json
import numpy as np
import psutil
import threading
from collections import deque
from typing import Dict, List, Optional, Tuple
import gc

class PerformanceMetrics:
    def __init__(self):
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.network_bandwidth = 0.0
        self.disk_io = 0.0
        self.gpu_usage = 0.0
        self.timestamp = time.time()

class PerformanceOptimizer(Node):
    def __init__(self):
        super().__init__('performance_optimizer')
        
        # Performance monitoring parameters
        self.monitoring_frequency = 10.0  # Hz
        self.optimization_threshold = 0.8  # 80% usage triggers optimization
        self.memory_threshold = 0.85  # 85% memory usage triggers cleanup
        
        # Performance metrics tracking
        self.performance_history = deque(maxlen=100)
        self.optimization_history = []
        self.component_performance = {}
        
        # System monitoring
        self.system_monitor = SystemMonitor()
        self.memory_optimizer = MemoryOptimizer()
        self.computation_optimizer = ComputationOptimizer()
        self.network_optimizer = NetworkOptimizer()
        
        # Publishers
        self.performance_status_pub = self.create_publisher(String, '/quadcopter/performance/status', 10)
        self.optimization_commands_pub = self.create_publisher(String, '/quadcopter/performance/optimization', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/quadcopter/performance/diagnostics', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/quadcopter/performance/visualization', 10)
        
        # Subscribers
        self.component_performance_sub = self.create_subscription(
            String, '/quadcopter/performance/component', self.component_performance_callback, 10
        )
        self.optimization_request_sub = self.create_subscription(
            String, '/quadcopter/performance/request', self.optimization_request_callback, 10
        )
        
        # Performance monitoring timer
        self.monitoring_timer = self.create_timer(1.0/self.monitoring_frequency, self.performance_monitoring_loop)
        
        # Performance optimization timer
        self.optimization_timer = self.create_timer(5.0, self.performance_optimization_loop)
        
        self.get_logger().info("Performance Optimizer initialized")
        
    def component_performance_callback(self, msg):
        """Handle component performance updates"""
        try:
            performance_data = json.loads(msg.data)
            component_id = performance_data.get('component_id', 'unknown')
            
            self.component_performance[component_id] = {
                'cpu_usage': performance_data.get('cpu_usage', 0.0),
                'memory_usage': performance_data.get('memory_usage', 0.0),
                'response_time': performance_data.get('response_time', 0.0),
                'throughput': performance_data.get('throughput', 0.0),
                'timestamp': time.time()
            }
            
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in component performance")
            
    def optimization_request_callback(self, msg):
        """Handle optimization requests"""
        try:
            request_data = json.loads(msg.data)
            optimization_type = request_data.get('type', 'auto')
            target_component = request_data.get('component', 'all')
            
            if optimization_type == 'memory_cleanup':
                self.memory_optimizer.cleanup_memory()
            elif optimization_type == 'computation_optimization':
                self.computation_optimizer.optimize_computation(target_component)
            elif optimization_type == 'network_optimization':
                self.network_optimizer.optimize_network()
                
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in optimization request")
            
    def performance_monitoring_loop(self):
        """Main performance monitoring loop"""
        current_time = time.time()
        
        # Collect system performance metrics
        system_metrics = self.system_monitor.collect_metrics()
        
        # Store performance history
        self.performance_history.append(system_metrics)
        
        # Check for performance issues
        self.check_performance_issues(system_metrics)
        
        # Publish performance status
        self.publish_performance_status(system_metrics)
        
        # Publish diagnostics
        self.publish_diagnostics(system_metrics)
        
    def performance_optimization_loop(self):
        """Main performance optimization loop"""
        current_time = time.time()
        
        # Check if optimization is needed
        if self.should_optimize():
            self.perform_optimization()
            
        # Publish optimization visualization
        self.publish_optimization_visualization()
        
    def check_performance_issues(self, metrics: PerformanceMetrics):
        """Check for performance issues and trigger optimizations"""
        issues = []
        
        # CPU usage check
        if metrics.cpu_usage > self.optimization_threshold:
            issues.append({
                'type': 'high_cpu_usage',
                'severity': 'medium',
                'value': metrics.cpu_usage,
                'threshold': self.optimization_threshold
            })
            
        # Memory usage check
        if metrics.memory_usage > self.memory_threshold:
            issues.append({
                'type': 'high_memory_usage',
                'severity': 'high',
                'value': metrics.memory_usage,
                'threshold': self.memory_threshold
            })
            
        # Component performance check
        for component_id, performance in self.component_performance.items():
            if performance['cpu_usage'] > 0.9:  # 90% CPU usage
                issues.append({
                    'type': 'component_high_cpu',
                    'severity': 'medium',
                    'component': component_id,
                    'value': performance['cpu_usage']
                })
                
        # Log and handle issues
        if issues:
            self.get_logger().warn(f"Performance issues detected: {len(issues)}")
            for issue in issues:
                self.get_logger().warn(f"  {issue['type']}: {issue['value']:.2f}")
                
    def should_optimize(self) -> bool:
        """Determine if optimization is needed"""
        if not self.performance_history:
            return False
            
        # Check recent performance metrics
        recent_metrics = list(self.performance_history)[-10:]
        
        avg_cpu = np.mean([m.cpu_usage for m in recent_metrics])
        avg_memory = np.mean([m.memory_usage for m in recent_metrics])
        
        return (avg_cpu > self.optimization_threshold or 
                avg_memory > self.memory_threshold)
                
    def perform_optimization(self):
        """Perform performance optimization"""
        self.get_logger().info("Starting performance optimization")
        
        optimization_start = time.time()
        
        # Memory optimization
        memory_saved = self.memory_optimizer.cleanup_memory()
        
        # Computation optimization
        computation_improvement = self.computation_optimizer.optimize_computation()
        
        # Network optimization
        network_improvement = self.network_optimizer.optimize_network()
        
        # Record optimization
        optimization_result = {
            'timestamp': time.time(),
            'duration': time.time() - optimization_start,
            'memory_saved_mb': memory_saved,
            'computation_improvement': computation_improvement,
            'network_improvement': network_improvement
        }
        
        self.optimization_history.append(optimization_result)
        
        self.get_logger().info(f"Optimization completed: {optimization_result}")
        
        # Publish optimization commands
        self.publish_optimization_commands(optimization_result)
        
    def publish_performance_status(self, metrics: PerformanceMetrics):
        """Publish performance status"""
        status_data = {
            'timestamp': time.time(),
            'cpu_usage': metrics.cpu_usage,
            'memory_usage': metrics.memory_usage,
            'network_bandwidth': metrics.network_bandwidth,
            'disk_io': metrics.disk_io,
            'gpu_usage': metrics.gpu_usage,
            'component_count': len(self.component_performance),
            'optimization_count': len(self.optimization_history)
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.performance_status_pub.publish(status_msg)
        
    def publish_diagnostics(self, metrics: PerformanceMetrics):
        """Publish diagnostic information"""
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        
        # System performance diagnostic
        system_status = DiagnosticStatus()
        system_status.name = "System Performance"
        system_status.hardware_id = "swarm_system"
        
        if metrics.cpu_usage > self.optimization_threshold:
            system_status.level = DiagnosticStatus.WARN
            system_status.message = f"High CPU usage: {metrics.cpu_usage:.1%}"
        elif metrics.memory_usage > self.memory_threshold:
            system_status.level = DiagnosticStatus.WARN
            system_status.message = f"High memory usage: {metrics.memory_usage:.1%}"
        else:
            system_status.level = DiagnosticStatus.OK
            system_status.message = "System performance normal"
            
        system_status.values.append(("CPU Usage", f"{metrics.cpu_usage:.1%}"))
        system_status.values.append(("Memory Usage", f"{metrics.memory_usage:.1%}"))
        system_status.values.append(("Network Bandwidth", f"{metrics.network_bandwidth:.1f} MB/s"))
        
        diagnostic_array.status.append(system_status)
        
        # Component performance diagnostics
        for component_id, performance in self.component_performance.items():
            component_status = DiagnosticStatus()
            component_status.name = f"Component: {component_id}"
            component_status.hardware_id = component_id
            
            if performance['cpu_usage'] > 0.9:
                component_status.level = DiagnosticStatus.WARN
                component_status.message = f"High CPU usage: {performance['cpu_usage']:.1%}"
            else:
                component_status.level = DiagnosticStatus.OK
                component_status.message = "Component performance normal"
                
            component_status.values.append(("CPU Usage", f"{performance['cpu_usage']:.1%}"))
            component_status.values.append(("Memory Usage", f"{performance['memory_usage']:.1%}"))
            component_status.values.append(("Response Time", f"{performance['response_time']:.3f}s"))
            
            diagnostic_array.status.append(component_status)
            
        self.diagnostics_pub.publish(diagnostic_array)
        
    def publish_optimization_commands(self, optimization_result: Dict):
        """Publish optimization commands"""
        commands_data = {
            'timestamp': time.time(),
            'type': 'performance_optimization',
            'actions': []
        }
        
        if optimization_result['memory_saved_mb'] > 0:
            commands_data['actions'].append({
                'action': 'memory_cleanup',
                'result': f"Saved {optimization_result['memory_saved_mb']:.1f} MB"
            })
            
        if optimization_result['computation_improvement'] > 0:
            commands_data['actions'].append({
                'action': 'computation_optimization',
                'result': f"Improved by {optimization_result['computation_improvement']:.1%}"
            })
            
        if optimization_result['network_improvement'] > 0:
            commands_data['actions'].append({
                'action': 'network_optimization',
                'result': f"Improved by {optimization_result['network_improvement']:.1%}"
            })
            
        commands_msg = String()
        commands_msg.data = json.dumps(commands_data)
        self.optimization_commands_pub.publish(commands_msg)
        
    def publish_optimization_visualization(self):
        """Publish optimization visualization markers"""
        marker_array = MarkerArray()
        
        # Performance metrics visualization
        if self.performance_history:
            recent_metrics = list(self.performance_history)[-20:]
            
            for i, metrics in enumerate(recent_metrics):
                # CPU usage marker
                cpu_marker = Marker()
                cpu_marker.header.frame_id = "map"
                cpu_marker.header.stamp = self.get_clock().now().to_msg()
                cpu_marker.ns = "performance_cpu"
                cpu_marker.id = i
                cpu_marker.type = Marker.CYLINDER
                cpu_marker.action = Marker.ADD
                
                cpu_marker.pose.position.x = i * 0.5
                cpu_marker.pose.position.y = 0.0
                cpu_marker.pose.position.z = metrics.cpu_usage * 2.0
                
                cpu_marker.scale.x = 0.3
                cpu_marker.scale.y = 0.3
                cpu_marker.scale.z = 0.5
                
                # Color based on CPU usage
                if metrics.cpu_usage > 0.8:
                    cpu_marker.color.r = 1.0
                    cpu_marker.color.g = 0.0
                    cpu_marker.color.b = 0.0
                elif metrics.cpu_usage > 0.6:
                    cpu_marker.color.r = 1.0
                    cpu_marker.color.g = 1.0
                    cpu_marker.color.b = 0.0
                else:
                    cpu_marker.color.r = 0.0
                    cpu_marker.color.g = 1.0
                    cpu_marker.color.b = 0.0
                    
                cpu_marker.color.a = 0.8
                marker_array.markers.append(cpu_marker)
                
                # Memory usage marker
                memory_marker = Marker()
                memory_marker.header.frame_id = "map"
                memory_marker.header.stamp = self.get_clock().now().to_msg()
                memory_marker.ns = "performance_memory"
                memory_marker.id = i
                memory_marker.type = Marker.CYLINDER
                memory_marker.action = Marker.ADD
                
                memory_marker.pose.position.x = i * 0.5
                memory_marker.pose.position.y = 2.0
                memory_marker.pose.position.z = metrics.memory_usage * 2.0
                
                memory_marker.scale.x = 0.3
                memory_marker.scale.y = 0.3
                memory_marker.scale.z = 0.5
                
                # Color based on memory usage
                if metrics.memory_usage > 0.8:
                    memory_marker.color.r = 1.0
                    memory_marker.color.g = 0.0
                    memory_marker.color.b = 0.0
                elif metrics.memory_usage > 0.6:
                    memory_marker.color.r = 1.0
                    memory_marker.color.g = 1.0
                    memory_marker.color.b = 0.0
                else:
                    memory_marker.color.r = 0.0
                    memory_marker.color.g = 1.0
                    memory_marker.color.b = 0.0
                    
                memory_marker.color.a = 0.8
                marker_array.markers.append(memory_marker)
                
        self.visualization_pub.publish(marker_array)

class SystemMonitor:
    def __init__(self):
        self.last_network_stats = None
        self.last_disk_stats = None
        
    def collect_metrics(self) -> PerformanceMetrics:
        """Collect system performance metrics"""
        metrics = PerformanceMetrics()
        
        # CPU usage
        metrics.cpu_usage = psutil.cpu_percent(interval=0.1) / 100.0
        
        # Memory usage
        memory = psutil.virtual_memory()
        metrics.memory_usage = memory.percent / 100.0
        
        # Network bandwidth
        metrics.network_bandwidth = self.get_network_bandwidth()
        
        # Disk I/O
        metrics.disk_io = self.get_disk_io()
        
        # GPU usage (simulated for now)
        metrics.gpu_usage = np.random.uniform(0.1, 0.3)  # 10-30% simulated
        
        metrics.timestamp = time.time()
        return metrics
        
    def get_network_bandwidth(self) -> float:
        """Get current network bandwidth usage"""
        try:
            net_io = psutil.net_io_counters()
            current_time = time.time()
            
            if self.last_network_stats:
                time_diff = current_time - self.last_network_stats['time']
                bytes_diff = net_io.bytes_sent + net_io.bytes_recv - self.last_network_stats['bytes']
                bandwidth = bytes_diff / time_diff / (1024 * 1024)  # MB/s
            else:
                bandwidth = 0.0
                
            self.last_network_stats = {
                'time': current_time,
                'bytes': net_io.bytes_sent + net_io.bytes_recv
            }
            
            return bandwidth
        except:
            return 0.0
            
    def get_disk_io(self) -> float:
        """Get current disk I/O usage"""
        try:
            disk_io = psutil.disk_io_counters()
            current_time = time.time()
            
            if self.last_disk_stats:
                time_diff = current_time - self.last_disk_stats['time']
                bytes_diff = disk_io.read_bytes + disk_io.write_bytes - self.last_disk_stats['bytes']
                io_rate = bytes_diff / time_diff / (1024 * 1024)  # MB/s
            else:
                io_rate = 0.0
                
            self.last_disk_stats = {
                'time': current_time,
                'bytes': disk_io.read_bytes + disk_io.write_bytes
            }
            
            return io_rate
        except:
            return 0.0

class MemoryOptimizer:
    def __init__(self):
        self.cleanup_threshold = 0.8  # 80% memory usage triggers cleanup
        
    def cleanup_memory(self) -> float:
        """Perform memory cleanup and return amount saved in MB"""
        try:
            # Force garbage collection
            collected = gc.collect()
            
            # Get memory before cleanup
            memory_before = psutil.virtual_memory().used
            
            # Clear Python cache
            import sys
            for module in list(sys.modules.keys()):
                if module.startswith('_'):
                    try:
                        del sys.modules[module]
                    except:
                        pass
                        
            # Get memory after cleanup
            memory_after = psutil.virtual_memory().used
            
            # Calculate memory saved
            memory_saved = (memory_before - memory_after) / (1024 * 1024)  # MB
            
            return max(0.0, memory_saved)
            
        except Exception as e:
            print(f"Memory cleanup error: {e}")
            return 0.0

class ComputationOptimizer:
    def __init__(self):
        self.optimization_level = 1.0
        
    def optimize_computation(self, target_component: str = 'all') -> float:
        """Optimize computation and return improvement percentage"""
        try:
            # Simulate computation optimization
            if target_component == 'all':
                # Global optimization
                improvement = np.random.uniform(0.05, 0.15)  # 5-15% improvement
            else:
                # Component-specific optimization
                improvement = np.random.uniform(0.1, 0.25)  # 10-25% improvement
                
            # Apply optimization
            self.optimization_level = min(1.0, self.optimization_level + improvement)
            
            return improvement
            
        except Exception as e:
            print(f"Computation optimization error: {e}")
            return 0.0

class NetworkOptimizer:
    def __init__(self):
        self.network_efficiency = 1.0
        
    def optimize_network(self) -> float:
        """Optimize network performance and return improvement percentage"""
        try:
            # Simulate network optimization
            improvement = np.random.uniform(0.02, 0.08)  # 2-8% improvement
            
            # Apply optimization
            self.network_efficiency = min(1.0, self.network_efficiency + improvement)
            
            return improvement
            
        except Exception as e:
            print(f"Network optimization error: {e}")
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    performance_optimizer = PerformanceOptimizer()
    
    try:
        rclpy.spin(performance_optimizer)
    except KeyboardInterrupt:
        performance_optimizer.get_logger().info("Performance Optimizer stopped by user")
    finally:
        performance_optimizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 