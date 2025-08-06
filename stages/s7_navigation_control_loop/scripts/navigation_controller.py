#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import numpy as np
import time
import math
from threading import Lock
from enum import Enum

class NavigationState(Enum):
    IDLE = "idle"
    FOLLOWING_PATH = "following_path"
    AVOIDING_OBSTACLE = "avoiding_obstacle"
    HOVERING = "hovering"
    LANDING = "landing"
    EMERGENCY = "emergency"

class PIDController:
    def __init__(self, kp, ki, kd, output_min=-1.0, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def compute(self, setpoint, measurement):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0
        error = setpoint - measurement
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))
        self.prev_error = error
        self.last_time = current_time
        return output

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Navigation parameters
        self.max_velocity = 2.0
        self.max_angular_velocity = 1.0
        self.hover_height = 2.0
        self.safety_distance = 1.5
        self.goal_tolerance = 0.5
        
        # Navigation state
        self.navigation_state = NavigationState.IDLE
        self.current_goal = None
        self.path_points = []
        self.current_path_index = 0
        
        # Position tracking
        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.goal_reached = False
        
        # PID controllers
        self.pid_x = PIDController(1.0, 0.1, 0.05)
        self.pid_y = PIDController(1.0, 0.1, 0.05)
        self.pid_z = PIDController(1.0, 0.1, 0.05)
        self.pid_yaw = PIDController(1.0, 0.1, 0.05)
        
        # Obstacle avoidance
        self.obstacles = []
        self.emergency_stop = False
        
        # Thread safety
        self.navigation_lock = Lock()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/quadcopter/cmd_vel', 10)
        self.navigation_status_pub = self.create_publisher(String, '/quadcopter/navigation/status', 10)
        self.path_marker_pub = self.create_publisher(MarkerArray, '/quadcopter/navigation/path', 10)
        
        # Subscribers
        self.mission_goal_sub = self.create_subscription(
            PoseStamped, '/quadcopter/mission/goal', 
            self.mission_goal_callback, 10
        )
        self.odometry_sub = self.create_subscription(
            Odometry, '/quadcopter/odometry', 
            self.odometry_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/quadcopter/scan', 
            self.scan_callback, 10
        )
        self.emergency_sub = self.create_subscription(
            Bool, '/quadcopter/emergency', 
            self.emergency_callback, 10
        )
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Navigation Controller initialized")
        
    def mission_goal_callback(self, msg):
        """Process mission goals from LLM Decision Engine"""
        with self.navigation_lock:
            self.current_goal = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'yaw': self.quaternion_to_yaw(msg.pose.orientation)
            }
            self.navigation_state = NavigationState.FOLLOWING_PATH
            self.get_logger().info(f"🎯 New goal: ({self.current_goal['x']:.1f}, {self.current_goal['y']:.1f}, {self.current_goal['z']:.1f})")
                
    def odometry_callback(self, msg):
        """Update current pose from odometry"""
        with self.navigation_lock:
            self.current_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
                'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
            }
            
    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        with self.navigation_lock:
            self.obstacles = []
            for i, range_measurement in enumerate(msg.ranges):
                if range_measurement < msg.range_min or range_measurement > msg.range_max:
                    continue
                if range_measurement < self.safety_distance:
                    angle = msg.angle_min + i * msg.angle_increment
                    x = range_measurement * math.cos(angle)
                    y = range_measurement * math.sin(angle)
                    world_x = self.current_pose['x'] + x * math.cos(self.current_pose['yaw']) - y * math.sin(self.current_pose['yaw'])
                    world_y = self.current_pose['y'] + x * math.sin(self.current_pose['yaw']) + y * math.cos(self.current_pose['yaw'])
                    self.obstacles.append({'x': world_x, 'y': world_y, 'distance': range_measurement})
                    
    def emergency_callback(self, msg):
        """Process emergency signals"""
        with self.navigation_lock:
            self.emergency_stop = msg.data
            if self.emergency_stop:
                self.navigation_state = NavigationState.EMERGENCY
                self.get_logger().warn("🚨 EMERGENCY STOP ACTIVATED")
                
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                          1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))
                          
    def check_goal_reached(self, goal):
        """Check if current goal has been reached"""
        if goal is None:
            return False
        distance = math.sqrt(
            (self.current_pose['x'] - goal['x'])**2 +
            (self.current_pose['y'] - goal['y'])**2 +
            (self.current_pose['z'] - goal['z'])**2
        )
        yaw_error = abs(self.current_pose['yaw'] - goal['yaw'])
        if yaw_error > math.pi:
            yaw_error = 2 * math.pi - yaw_error
        return distance < self.goal_tolerance and yaw_error < 0.1
        
    def compute_control_commands(self):
        """Compute control commands based on current state"""
        cmd_vel = Twist()
        
        if self.navigation_state == NavigationState.EMERGENCY:
            return cmd_vel
            
        if self.navigation_state == NavigationState.HOVERING:
            cmd_vel.linear.x = self.pid_x.compute(self.current_pose['x'], self.current_pose['x'])
            cmd_vel.linear.y = self.pid_y.compute(self.current_pose['y'], self.current_pose['y'])
            cmd_vel.linear.z = self.pid_z.compute(self.hover_height, self.current_pose['z'])
            cmd_vel.angular.z = 0.0
            
        elif self.navigation_state == NavigationState.FOLLOWING_PATH:
            if self.current_goal is None:
                self.navigation_state = NavigationState.HOVERING
                return self.compute_control_commands()
                
            if self.obstacles and any(obs['distance'] < self.safety_distance * 0.5 for obs in self.obstacles):
                self.navigation_state = NavigationState.AVOIDING_OBSTACLE
                return self.compute_control_commands()
                
            if self.check_goal_reached(self.current_goal):
                self.get_logger().info("✅ Goal reached!")
                self.navigation_state = NavigationState.HOVERING
                self.current_goal = None
                return self.compute_control_commands()
                
            cmd_vel.linear.x = self.pid_x.compute(self.current_goal['x'], self.current_pose['x'])
            cmd_vel.linear.y = self.pid_y.compute(self.current_goal['y'], self.current_pose['y'])
            cmd_vel.linear.z = self.pid_z.compute(self.current_goal['z'], self.current_pose['z'])
            cmd_vel.angular.z = self.pid_yaw.compute(self.current_goal['yaw'], self.current_pose['yaw'])
            
        elif self.navigation_state == NavigationState.AVOIDING_OBSTACLE:
            avoidance_goal = self.compute_avoidance_goal()
            if avoidance_goal:
                cmd_vel.linear.x = self.pid_x.compute(avoidance_goal['x'], self.current_pose['x'])
                cmd_vel.linear.y = self.pid_y.compute(avoidance_goal['y'], self.current_pose['y'])
                cmd_vel.linear.z = self.pid_z.compute(avoidance_goal['z'], self.current_pose['z'])
                cmd_vel.angular.z = self.pid_yaw.compute(avoidance_goal['yaw'], self.current_pose['yaw'])
            else:
                self.navigation_state = NavigationState.HOVERING
                return self.compute_control_commands()
                
        cmd_vel.linear.x = max(-self.max_velocity, min(self.max_velocity, cmd_vel.linear.x))
        cmd_vel.linear.y = max(-self.max_velocity, min(self.max_velocity, cmd_vel.linear.y))
        cmd_vel.linear.z = max(-self.max_velocity, min(self.max_velocity, cmd_vel.linear.z))
        cmd_vel.angular.z = max(-self.max_angular_velocity, min(self.max_angular_velocity, cmd_vel.angular.z))
        
        return cmd_vel
        
    def compute_avoidance_goal(self):
        """Compute avoidance goal to avoid obstacles"""
        if not self.obstacles or self.current_goal is None:
            return self.current_goal
            
        closest_obstacle = min(self.obstacles, key=lambda obs: obs['distance'])
        if closest_obstacle['distance'] > 2.0:
            return self.current_goal
            
        dx = closest_obstacle['x'] - self.current_pose['x']
        dy = closest_obstacle['y'] - self.current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > 0:
            dx = dx / distance * 2.0
            dy = dy / distance * 2.0
            
        avoidance_goal = {
            'x': self.current_pose['x'] - dx,
            'y': self.current_pose['y'] - dy,
            'z': self.current_goal['z'],
            'yaw': self.current_goal['yaw']
        }
        
        return avoidance_goal
        
    def publish_navigation_status(self):
        """Publish navigation status"""
        status_msg = String()
        status_data = {
            'state': self.navigation_state.value,
            'current_pose': self.current_pose,
            'current_goal': self.current_goal,
            'goal_reached': self.goal_reached,
            'emergency_stop': self.emergency_stop,
            'obstacle_count': len(self.obstacles)
        }
        status_msg.data = str(status_data)
        self.navigation_status_pub.publish(status_msg)
        
    def control_loop(self):
        """Main navigation control loop"""
        with self.navigation_lock:
            cmd_vel = self.compute_control_commands()
            self.cmd_vel_pub.publish(cmd_vel)
            self.publish_navigation_status()
            
            if self.navigation_state != NavigationState.IDLE:
                self.get_logger().debug(f"Navigation: {self.navigation_state.value}, "
                                      f"Goal: {self.current_goal is not None}, "
                                      f"Obstacles: {len(self.obstacles)}")

def main(args=None):
    rclpy.init(args=args)
    navigation_controller = NavigationController()
    
    try:
        rclpy.spin(navigation_controller)
    except KeyboardInterrupt:
        navigation_controller.get_logger().info("Navigation Controller stopped by user")
    finally:
        navigation_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 