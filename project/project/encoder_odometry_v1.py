#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import math
import time
import threading
from gpiozero import RotaryEncoder

class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')
        
        # Hard-coded parameters for standalone mode
        self.wheel_diameter = 0.07  # 70mm wheel diameter
        self.wheel_separation = 0.18  # Distance between wheels in meters
        self.encoder_ppr = 1358  # Pulses per revolution (from OE-28 datasheet)
        self.encoder_mode = 4  # Quadrature encoding (x4 pulses)
        self.update_rate = 10.0  # Hz
        
        # Calculate derived values
        self.wheel_radius = self.wheel_diameter / 2.0
        self.ticks_per_revolution = self.encoder_ppr * self.encoder_mode
        self.ticks_per_meter = self.ticks_per_revolution / (math.pi * self.wheel_diameter)
        
        # Log important configuration details
        print(f"Wheel diameter: {self.wheel_diameter}m")
        print(f"Wheel separation: {self.wheel_separation}m")
        print(f"Encoder PPR: {self.encoder_ppr} with mode x{self.encoder_mode}")
        print(f"Ticks per meter: {self.ticks_per_meter}")
        
        # Create publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Encoder setup - using GPIO pins matching the OE-28 connection
        # Based on Raspberry Pi GPIO layout
        # Left encoder (A, B)
        self.left_encoder = RotaryEncoder(6, 5, max_steps=0)
        # Right encoder (A, B)
        self.right_encoder = RotaryEncoder(24, 25, max_steps=0)
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous encoder values for delta calculation
        self.prev_left_steps = 0
        self.prev_right_steps = 0
        self.last_time = self.get_clock().now()
        
        # Debug counters
        self.debug_counter = 0
        
        # Create timer for odometry updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_odometry)
        
        print("Encoder odometry node initialized")
    
    def cmd_vel_callback(self, msg):
        # This is a placeholder for motor control code
        # You would use this to control the motors based on desired velocity
        pass
    
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        
        # Get current encoder values
        current_left_steps = self.left_encoder.steps
        current_right_steps = self.right_encoder.steps
        
        # Calculate delta steps
        delta_left = current_left_steps - self.prev_left_steps
        delta_right = current_right_steps - self.prev_right_steps
        
        # Update previous values
        self.prev_left_steps = current_left_steps
        self.prev_right_steps = current_right_steps
        
        # Skip update if time delta is too small
        if dt < 0.001:
            return
            
        # Convert ticks to distance (meters)
        left_distance = delta_left / self.ticks_per_meter
        right_distance = delta_right / self.ticks_per_meter
        
        # Calculate robot movement
        distance = (left_distance + right_distance) / 2.0
        rotation = (right_distance - left_distance) / self.wheel_separation
        
        # Calculate velocities
        linear_velocity = distance / dt
        angular_velocity = rotation / dt
        
        # Update pose
        if abs(rotation) < 0.0001:  # Going straight
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
        else:  # Following an arc
            radius = distance / rotation
            self.x += radius * (math.sin(self.theta + rotation) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + rotation) - math.cos(self.theta))
        
        self.theta += rotation
        # Normalize theta to -pi to +pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Debug output (every 5 seconds)
        self.debug_counter += 1
        if self.debug_counter % (5 * self.update_rate) == 0:
            print(f"Left encoder: {current_left_steps}, Right encoder: {current_right_steps}")
            print(f"Position: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")
            print(f"Velocities: linear={linear_velocity:.3f}, angular={angular_velocity:.3f}")
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion)
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Set velocity
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity
        
        # Add covariance - simplified approach with moderate uncertainty
        # Pose covariance [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance[0] = 0.01  # x
        odom_msg.pose.covariance[7] = 0.01  # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        
        # Twist covariance [vx, vy, vz, wx, wy, wz]
        odom_msg.twist.covariance[0] = 0.01  # vx
        odom_msg.twist.covariance[35] = 0.01  # wz
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
        
        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = math.sin(self.theta / 2.0)
        tf_msg.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(tf_msg)
        
        # Update time
        self.last_time = current_time

def main():
    # Initialize ROS
    rclpy.init()
    
    # Create node
    node = EncoderOdometry()
    
    print("Starting encoder odometry node")
    
    try:
        # Run node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping encoder odometry node")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
