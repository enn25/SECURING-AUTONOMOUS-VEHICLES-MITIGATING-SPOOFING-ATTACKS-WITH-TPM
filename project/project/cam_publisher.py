#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'rpi/camera/image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.bridge = CvBridge()
        
        # Initialize the camera using OpenCV
        # Try different camera indices if 0 doesn't work (0, 1, or 2)
        self.cap = cv2.VideoCapture(0)
        
        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            raise RuntimeError("Failed to open camera")
        
        time.sleep(2)  # Give the camera time to initialize
        self.get_logger().info('Camera publisher node initialized')

    def timer_callback(self):
        # Capture frame
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
            
        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_frame"
        
        # Publish image
        self.publisher_.publish(img_msg)
        self.get_logger().debug('Publishing camera frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
