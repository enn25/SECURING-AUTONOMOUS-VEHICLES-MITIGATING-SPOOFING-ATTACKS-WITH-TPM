import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
import threading
from gpiozero import RotaryEncoder
import math

# --- Encoder & Wheel Specs ---
GEARED_CPR = 6300  # 7 counts per motor rev * 900 gear ratio
WHEEL_DIAMETER = 0.07  # meters
WHEEL_RADIUS = WHEEL_DIAMETER / 2
ENCODER_TICKS_PER_METER = GEARED_CPR / (math.pi * WHEEL_DIAMETER)  # ~28642

# --- Encoder Pins ---
encoderA = RotaryEncoder(6, 5)    # Left Wheel Encoder (Channel A and B)
encoderB = RotaryEncoder(24, 25)  # Right Wheel Encoder (Channel A and B)

# --- Encoder Counters ---
encoderA_steps = 0
encoderB_steps = 0
lock = threading.Lock()

# --- Encoder Callback Functions ---
def update_encoderA():
    global encoderA_steps
    with lock:
        if encoderA.value == 1:
            encoderA_steps += 1
        else:
            encoderA_steps -= 1

def update_encoderB():
    global encoderB_steps
    with lock:
        if encoderB.value == 1:
            encoderB_steps += 1
        else:
            encoderB_steps -= 1

# Assign callbacks
encoderA.when_rotated = update_encoderA
encoderB.when_rotated = update_encoderB

class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initial Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # Start monitoring encoder changes
        self.monitor_thread = threading.Thread(target=self.monitor_encoders, daemon=True)
        self.monitor_thread.start()

    def monitor_encoders(self):
        global encoderA_steps, encoderB_steps

        while rclpy.ok():
            time.sleep(0.1)  # 10 Hz update rate
            with lock:
                left_ticks = encoderA_steps
                right_ticks = encoderB_steps
                encoderA_steps = 0
                encoderB_steps = 0

            # Convert to distances
            left_distance = left_ticks / ENCODER_TICKS_PER_METER
            right_distance = right_ticks / ENCODER_TICKS_PER_METER
            distance = (left_distance + right_distance) / 2.0
            dtheta = (right_distance - left_distance) / WHEEL_DIAMETER

            # Update pose
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
            self.theta += dtheta

            self.publish_odometry(distance, dtheta)

    def publish_odometry(self, distance, dtheta):
        now = self.get_clock().now().to_msg()

        # Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocities (approximate)
        odom_msg.twist.twist.linear.x = distance / 0.1
        odom_msg.twist.twist.angular.z = dtheta / 0.1

        self.odom_pub.publish(odom_msg)

        # TF Transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
