#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class EncoderTest(Node):
    def __init__(self):
        super().__init__('encoder_test')
        
        # Parameters
        self.test_distance = 1.0  # meters to move forward
        self.speed = 0.1  # m/s linear velocity
        self.timeout = 30.0  # seconds before giving up
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # State variables
        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None
        self.is_moving = False
        self.test_complete = False
        
        # Timer for the test sequence
        self.timer = self.create_timer(0.1, self.test_sequence)
        
        self.get_logger().info('Encoder Test Node initialized')
        self.get_logger().info(f'Will move {self.test_distance} meters forward')
    
    def odom_callback(self, msg):
        # Store position from odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Set start position if not already set
        if self.start_x is None and self.start_y is None and self.is_moving:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f'Starting position: x={self.start_x:.3f}, y={self.start_y:.3f}')
    
    def test_sequence(self):
        if self.test_complete:
            return
            
        if not self.is_moving:
            # Start moving forward
            self.get_logger().info('Starting movement test')
            self.is_moving = True
            self.move_forward()
            self.start_time = time.time()
            return
            
        # Check if we have odometry data
        if self.current_x is None or self.start_x is None:
            return
            
        # Calculate distance traveled
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        distance_traveled = math.sqrt(dx**2 + dy**2)
        
        # Check if we've reached the target distance
        if distance_traveled >= self.test_distance:
            self.stop_moving()
            self.get_logger().info(f'Target distance reached!')
            self.get_logger().info(f'Final position: x={self.current_x:.3f}, y={self.current_y:.3f}')
            self.get_logger().info(f'Distance traveled: {distance_traveled:.3f} meters')
            self.get_logger().info(f'Target distance: {self.test_distance:.3f} meters')
            self.get_logger().info(f'Error: {(distance_traveled - self.test_distance):.3f} meters')
            self.get_logger().info(f'Error percentage: {((distance_traveled - self.test_distance) / self.test_distance * 100):.2f}%')
            self.test_complete = True
            return
            
        # Timeout check
        if time.time() - self.start_time > self.timeout:
            self.stop_moving()
            self.get_logger().info(f'Timeout reached!')
            self.get_logger().info(f'Distance traveled: {distance_traveled:.3f} meters')
            self.get_logger().info(f'Target distance: {self.test_distance:.3f} meters')
            self.test_complete = True
            return
            
        # Log progress every second
        if int(time.time()) % 2 == 0:
            self.get_logger().info(f'Distance traveled: {distance_traveled:.3f} meters')
    
    def move_forward(self):
        """Send command to move robot forward"""
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Moving forward at {self.speed} m/s')
    
    def stop_moving(self):
        """Send command to stop robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Stopping')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the robot stops
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.get_logger().info('Test ended, robot stopped')
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
