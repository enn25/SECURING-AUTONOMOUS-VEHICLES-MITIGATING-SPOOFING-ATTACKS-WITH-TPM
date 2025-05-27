import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import socket


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_udp_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Set up UDP socket
        UDP_IP = "0.0.0.0"  # Listen on all interfaces
        UDP_PORT = 5555

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.01, self.publish_imu)

    def publish_imu(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            decoded = data.decode().strip()
            print(f"Received: {decoded}")

            parts = decoded.split(',')
            if len(parts) < 13:
                self.get_logger().warn(f'Incomplete IMU data received: {decoded}')
                return

            # Remove whitespace and convert
            parts = [p.strip() for p in parts]

            # Parse accelerometer
            accel = list(map(float, parts[2:5]))  # skip marker '3'

            # Parse gyroscope
            gyro = list(map(float, parts[6:9]))   # skip marker '4'

            # Parse magnetometer (not used in Imu message)
            mag = list(map(float, parts[10:13]))  # skip marker '5'

            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.linear_acceleration = Vector3(x=accel[0], y=accel[1], z=accel[2])
            imu_msg.angular_velocity = Vector3(x=gyro[0], y=gyro[1], z=gyro[2])
            imu_msg.orientation_covariance[0] = -1.0  # Orientation unknown

            self.publisher_.publish(imu_msg)

        except BlockingIOError:
            pass  # No data yet


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
