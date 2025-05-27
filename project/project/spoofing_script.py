print("Spoofing script started")
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import copy
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class LidarSpoofingNode(Node):
    def __init__(self):
        super().__init__('lidar_spoofing_node')
        
        # Create a QoS profile that's more compatible with sensor data
        # Use sensor data profile which typically uses best effort reliability
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Create a subscription to the original scan topic with sensor data QoS
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos)  # Use the sensor data QoS profile
            
        # Create a publisher for spoofed data with the same QoS
        self.spoofed_publisher = self.create_publisher(
            LaserScan,
            '/scan_spoofed',
            qos_profile=qos)
            
        # Also republish the original data for comparison with the same QoS
        self.original_publisher = self.create_publisher(
            LaserScan,
            '/scan_original',
            qos_profile=qos)
            
        # Spoofing parameters - EXTREMELY aggressive for testing
        self.spoofing_enabled = True
        self.spoofing_probability = 1.0  # Always spoof
        self.noise_level = 2.0  # Much larger noise (in meters)
        
        # Spoofing types and their weights (probability distribution)
        self.spoofing_types = {
            'random_noise': 0.25,       # Add random noise to ranges
            'ghost_objects': 0.25,      # Add non-existent objects
            'object_removal': 0.25,     # Remove existing objects
            'range_modification': 0.25  # Modify ranges of existing objects
        }
        
        self.get_logger().info('LiDAR Spoofing Node initialized with AGGRESSIVE modifications')
        self.get_logger().info('Listening on topic: /scan with BEST_EFFORT reliability QoS')
        
        # Create a timer for parameter update and status checking
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # For demonstration, track statistics
        self.total_scans = 0
        self.spoofed_scans = 0
        self.last_receive_time = None
        
    def timer_callback(self):
        """Display current spoofing statistics and connection status"""
        now = time.time()
        
        if self.last_receive_time:
            time_since_last = now - self.last_receive_time
            self.get_logger().info(f'Time since last message: {time_since_last:.2f} seconds')
            
            # Alert if no messages for more than 5 seconds
            if time_since_last > 5.0:
                self.get_logger().warn('No messages received for over 5 seconds! Check topic connections.')
        else:
            if self.total_scans == 0:  # If no messages have been received yet
                self.get_logger().warn('No scan messages received yet! Check your topic name and QoS settings.')
                self.get_logger().info('To check available topics: ros2 topic list')
                self.get_logger().info('To check topic QoS: ros2 topic info /scan')
        
        if self.total_scans > 0:
            spoofing_rate = (self.spoofed_scans / self.total_scans) * 100
            self.get_logger().info(f'Spoofing Statistics: {self.spoofed_scans}/{self.total_scans} scans spoofed ({spoofing_rate:.1f}%)')
    
    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.total_scans += 1
        self.last_receive_time = time.time()
        
        self.get_logger().info(f'Received scan message #{self.total_scans}, with {len(msg.ranges)} range readings')
        
        # Always republish the original for comparison
        original_msg = copy.deepcopy(msg)
        self.original_publisher.publish(original_msg)
        
        # Create a copy for spoofing
        spoofed_msg = copy.deepcopy(msg)
        
        # Now always apply spoofing for debugging
        if self.spoofing_enabled:
            self.spoofed_scans += 1
            spoofed_msg = self.apply_aggressive_spoofing(spoofed_msg)
            self.get_logger().info('Applied aggressive spoofing to scan')
        
        # Publish the spoofed message
        self.spoofed_publisher.publish(spoofed_msg)
        
        # Print a sample of ranges for comparison
        sample_idx = len(msg.ranges) // 2
        original_sample = msg.ranges[sample_idx] if sample_idx < len(msg.ranges) else 0
        spoofed_sample = spoofed_msg.ranges[sample_idx] if sample_idx < len(spoofed_msg.ranges) else 0
        self.get_logger().info(f'Sample range comparison - Original: {original_sample}, Spoofed: {spoofed_sample}')
    
    def apply_aggressive_spoofing(self, msg):
        """Apply aggressive spoofing to make changes very obvious"""
        # Convert tuple to list so we can modify it
        ranges = list(msg.ranges)
        
        # Choose a spoofing method based on weights
        spoofing_type = random.choices(
            list(self.spoofing_types.keys()),
            weights=list(self.spoofing_types.values()),
            k=1
        )[0]
        
        self.get_logger().info(f"Using spoofing type: {spoofing_type}")
        
        # Apply the chosen spoofing method with aggressive parameters
        if spoofing_type == 'random_noise':
            # Add large random noise to many points
            for i in range(len(ranges)):
                # Check if it's a valid range value (not 0.0 or '...')
                if isinstance(ranges[i], (int, float)) and ranges[i] > 0.0 and ranges[i] < msg.range_max:
                    if random.random() < 0.8:  # 80% of valid points get noise
                        noise = random.uniform(-self.noise_level, self.noise_level)
                        ranges[i] = max(msg.range_min, min(msg.range_max, ranges[i] + noise))
                    
        elif spoofing_type == 'ghost_objects':
            # Add multiple fake objects
            for _ in range(3):  # Add 3 ghost objects
                section_start = random.randint(0, len(ranges) - 30)
                section_length = random.randint(10, 30)
                fake_distance = random.uniform(0.5, 3.0)
                
                for i in range(section_start, section_start + section_length):
                    if i < len(ranges) and isinstance(ranges[i], (int, float)):
                        ranges[i] = fake_distance + random.uniform(-0.1, 0.1)
                
                self.get_logger().info(f"Added ghost object at indices {section_start}-{section_start+section_length} at distance {fake_distance:.2f}m")
                
        elif spoofing_type == 'object_removal':
            # Find and remove large sections of valid readings
            valid_sections = []
            current_section = []
            
            # Identify valid sections (likely objects)
            for i, r in enumerate(ranges):
                if isinstance(r, (int, float)) and r > 0.0 and r < msg.range_max:
                    current_section.append(i)
                elif current_section:
                    if len(current_section) > 5:  # Only consider sections with at least 5 points
                        valid_sections.append(current_section)
                    current_section = []
            
            # Add the last section if it exists
            if current_section and len(current_section) > 5:
                valid_sections.append(current_section)
            
            # Remove random sections (objects)
            if valid_sections:
                sections_to_remove = random.sample(valid_sections, min(3, len(valid_sections)))
                for section in sections_to_remove:
                    for i in section:
                        ranges[i] = 0.0  # Set to invalid reading
                    self.get_logger().info(f"Removed object at indices {section[0]}-{section[-1]}")
                
        elif spoofing_type == 'range_modification':
            # Drastically modify ranges of existing objects
            for i in range(len(ranges)):
                if isinstance(ranges[i], (int, float)) and ranges[i] > 0.0 and ranges[i] < msg.range_max:
                    mod_type = random.choice([0, 1, 2])
                    
                    if mod_type == 0:
                        # Double the distance
                        ranges[i] = min(msg.range_max, ranges[i] * 2.0)
                    elif mod_type == 1:
                        # Halve the distance
                        ranges[i] = max(msg.range_min, ranges[i] / 2.0)
                    else:
                        # Invert the distance within range bounds
                        ranges[i] = msg.range_max - ranges[i] + msg.range_min
        
        # Apply additional extreme modification
        # Create a completely fake wall or object in a random direction
        section_start = random.randint(0, len(ranges) - 40)
        section_length = min(40, len(ranges) - section_start)
        fake_distance = random.uniform(0.5, 4.0)
        
        for i in range(section_start, section_start + section_length):
            if i < len(ranges):
                # Create a smooth curved pattern for the fake wall
                curve_factor = abs((i - section_start) - (section_length / 2)) / max(1, (section_length / 2))
                distance_variation = curve_factor * 0.5  # up to 0.5m variation
                ranges[i] = fake_distance + distance_variation
        
        self.get_logger().info(f"Added extreme fake wall at indices {section_start}-{section_start+section_length}")
        
        # Update the message with modified ranges
        msg.ranges = ranges
        return msg
    
    def set_spoofing_parameters(self, enabled=None, probability=None, noise_level=None):
        """Update spoofing parameters (could be exposed as ROS2 parameters)"""
        if enabled is not None:
            self.spoofing_enabled = enabled
            self.get_logger().info(f'Spoofing enabled: {enabled}')
            
        if probability is not None:
            self.spoofing_probability = max(0.0, min(1.0, probability))
            self.get_logger().info(f'Spoofing probability: {self.spoofing_probability}')
            
        if noise_level is not None:
            self.noise_level = max(0.0, noise_level)
            self.get_logger().info(f'Noise level: {self.noise_level}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarSpoofingNode()
    
    try:
        # Add a header message to indicate the script is running
        print("Spoofing script started")
        node.get_logger().info("Node is running, press Ctrl+C to terminate")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly by user interrupt')
    except Exception as e:
        node.get_logger().error(f'Error during node execution: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
