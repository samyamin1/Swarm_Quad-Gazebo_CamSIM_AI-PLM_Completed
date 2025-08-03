#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
import numpy as np
import math
from enum import Enum

class FlightMode(Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    HOLD = "HOLD"
    LAND = "LAND"
    TAKEOFF = "TAKEOFF"

class PIDController:
    def __init__(self, kp, ki, kd, output_min=-1.0, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.prev_error = 0.0
        self.integral = 0.0
        
    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        
        # Total output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.output_min, min(self.output_max, output))
        
        self.prev_error = error
        return output

class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')
        
        # Flight state
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.current_altitude = 0.0
        self.flight_mode = FlightMode.MANUAL
        
        # PID controllers
        self.roll_pid = PIDController(1.0, 0.1, 0.05)
        self.pitch_pid = PIDController(1.0, 0.1, 0.05)
        self.yaw_pid = PIDController(2.0, 0.1, 0.1)
        self.throttle_pid = PIDController(1.0, 0.1, 0.05)
        
        # Setpoints
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_setpoint = 0.0
        self.throttle_setpoint = 0.0
        self.altitude_setpoint = 0.0
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 'quadcopter/sensors/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, 'quadcopter/sensors/gps', self.gps_callback, 10)
        self.altitude_sub = self.create_subscription(
            LaserScan, 'quadcopter/sensors/altitude', self.altitude_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'quadcopter/state/odometry', self.odom_callback, 10)
        
        # Control command subscribers
        self.velocity_cmd_sub = self.create_subscription(
            Twist, 'quadcopter/control/velocity', self.velocity_command_callback, 10)
        self.waypoint_cmd_sub = self.create_subscription(
            PoseStamped, 'quadcopter/control/waypoint', self.waypoint_command_callback, 10)
        self.mode_cmd_sub = self.create_subscription(
            String, 'quadcopter/control/mode', self.mode_command_callback, 10)
        
        # Publishers
        self.motor_cmd_pub = self.create_publisher(
            Twist, 'quadcopter/control/motors', 10)
        self.state_pub = self.create_publisher(
            String, 'quadcopter/state/flight_mode', 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz
        
        # Time tracking
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Flight controller initialized')
    
    def imu_callback(self, msg):
        """Handle IMU data"""
        # Extract roll, pitch, yaw from orientation
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Update current state
        self.current_velocity.twist.angular.x = msg.angular_velocity.x
        self.current_velocity.twist.angular.y = msg.angular_velocity.y
        self.current_velocity.twist.angular.z = msg.angular_velocity.z
    
    def gps_callback(self, msg):
        """Handle GPS data"""
        # Update position
        self.current_pose.pose.position.x = msg.longitude
        self.current_pose.pose.position.y = msg.latitude
        self.current_pose.pose.position.z = msg.altitude
    
    def altitude_callback(self, msg):
        """Handle altitude sensor data"""
        if len(msg.ranges) > 0:
            self.current_altitude = msg.ranges[0]
    
    def odom_callback(self, msg):
        """Handle odometry data"""
        self.current_pose = msg.pose
        self.current_velocity = msg.twist
    
    def velocity_command_callback(self, msg):
        """Handle velocity commands"""
        if self.flight_mode == FlightMode.MANUAL:
            # Direct velocity control
            self.roll_setpoint = msg.angular.x
            self.pitch_setpoint = msg.angular.y
            self.yaw_setpoint = msg.angular.z
            self.throttle_setpoint = msg.linear.z
    
    def waypoint_command_callback(self, msg):
        """Handle waypoint commands"""
        if self.flight_mode == FlightMode.AUTO:
            # Convert waypoint to velocity commands
            target_x = msg.pose.position.x
            target_y = msg.pose.position.y
            target_z = msg.pose.position.z
            
            # Simple proportional control for waypoint following
            dx = target_x - self.current_pose.pose.position.x
            dy = target_y - self.current_pose.pose.position.y
            dz = target_z - self.current_pose.pose.position.z
            
            # Set velocity setpoints
            self.roll_setpoint = dy * 0.5  # Proportional control
            self.pitch_setpoint = -dx * 0.5
            self.throttle_setpoint = dz * 0.5
    
    def mode_command_callback(self, msg):
        """Handle flight mode commands"""
        try:
            self.flight_mode = FlightMode(msg.data)
            self.get_logger().info(f'Flight mode changed to: {self.flight_mode.value}')
        except ValueError:
            self.get_logger().warn(f'Invalid flight mode: {msg.data}')
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def control_loop(self):
        """Main control loop"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Get current state
        current_roll = 0.0  # Extract from IMU
        current_pitch = 0.0
        current_yaw = 0.0
        current_throttle = self.current_velocity.twist.linear.z
        
        # Compute control outputs
        roll_output = self.roll_pid.compute(self.roll_setpoint, current_roll, dt)
        pitch_output = self.pitch_pid.compute(self.pitch_setpoint, current_pitch, dt)
        yaw_output = self.yaw_pid.compute(self.yaw_setpoint, current_yaw, dt)
        throttle_output = self.throttle_pid.compute(self.throttle_setpoint, current_throttle, dt)
        
        # Convert to motor commands (simplified quadcopter mixing)
        motor_fl = throttle_output + roll_output + pitch_output + yaw_output
        motor_fr = throttle_output - roll_output + pitch_output - yaw_output
        motor_rl = throttle_output + roll_output - pitch_output - yaw_output
        motor_rr = throttle_output - roll_output - pitch_output + yaw_output
        
        # Clamp motor outputs
        motor_fl = max(0.0, min(1.0, motor_fl))
        motor_fr = max(0.0, min(1.0, motor_fr))
        motor_rl = max(0.0, min(1.0, motor_rl))
        motor_rr = max(0.0, min(1.0, motor_rr))
        
        # Publish motor commands
        motor_cmd = Twist()
        motor_cmd.linear.x = motor_fl
        motor_cmd.linear.y = motor_fr
        motor_cmd.linear.z = motor_rl
        motor_cmd.angular.x = motor_rr
        self.motor_cmd_pub.publish(motor_cmd)
        
        # Publish current state
        state_msg = String()
        state_msg.data = self.flight_mode.value
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = FlightController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 