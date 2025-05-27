import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class ImuUSBPublisher(Node):
    def __init__(self):
        super().__init__('imu_usb_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)

        # Update this if ttyACM0 is different
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_and_publish)

    def read_and_publish(self):
        try:
            line = self.serial_port.readline().decode().strip()
            if not line:
                return

            # Expecting CSV: ax,ay,az,gx,gy,gz
            data = list(map(float, line.split(',')))
            if len(data) != 6:
                self.get_logger().warn(f"Unexpected data format: {line}")
                return

            ax, ay, az, gx, gy, gz = data
            imu_msg = Imu()

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            imu_msg.header.frame_id = "imu_link"
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.orientation_covariance[0] = -1.0  # Orientation unknown

            self.publisher_.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuUSBPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
