import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess

class ScriptRunner(Node):
    def __init__(self):
        super().__init__('script_runner')

        # Subscribers to listen for toggle commands
        self.create_subscription(Bool, '/toggle_spoofing', self.toggle_spoofing_callback, 10)
        self.create_subscription(Bool, '/toggle_detection', self.toggle_detection_callback, 10)

        self.spoofing_process = None
        self.detection_process = None
        self.get_logger().info("Script Runner Node Initialized!")

    def toggle_spoofing_callback(self, msg):
        if msg.data:
            self.get_logger().info("Starting LiDAR Spoofing Script")
            self.spoofing_process = subprocess.Popen(['python3', 'spoofing_script.py'])
        else:
            self.get_logger().info("Stopping LiDAR Spoofing Script")
            if self.spoofing_process:
                self.spoofing_process.terminate()

    def toggle_detection_callback(self, msg):
        if msg.data:
            self.get_logger().info("Starting Spoof Detection Script")
            self.detection_process = subprocess.Popen(['python3', 'detection_script.py'])
        else:
            self.get_logger().info("Stopping Spoof Detection Script")
            if self.detection_process:
                self.detection_process.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = ScriptRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
