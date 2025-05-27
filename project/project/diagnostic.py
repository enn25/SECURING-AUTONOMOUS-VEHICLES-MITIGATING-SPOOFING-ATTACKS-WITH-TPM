import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil
import time
import subprocess

class DiagnosticPublisher(Node):
    def __init__(self):
        super().__init__('diagnostic_publisher')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)

    def read_voltage(self):
        try:
            result = subprocess.check_output(['vcgencmd', 'measure_volts', 'core']).decode('utf-8')
            voltage = float(result.replace('volt=', '').replace('V\n', '').strip())
            return voltage
        except Exception as e:
            self.get_logger().error(f'Error reading voltage: {e}')
            return 0.0

    def publish_diagnostics(self):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        # CPU Usage (Smoothing by taking average over 1 second)
        cpu_status = DiagnosticStatus()
        cpu_status.name = 'CPU Usage'
        cpu_percent = psutil.cpu_percent(interval=1)
        cpu_status.level = DiagnosticStatus.OK if cpu_percent < 80 else DiagnosticStatus.WARN
        cpu_status.message = 'Normal' if cpu_status.level == DiagnosticStatus.OK else 'High CPU Load'
        cpu_status.values.append(KeyValue(key='CPU %', value=str(cpu_percent)))

        # Memory Usage
        memory_status = DiagnosticStatus()
        memory_status.name = 'Memory Usage'
        mem_info = psutil.virtual_memory()
        memory_status.level = DiagnosticStatus.OK if mem_info.percent < 80 else DiagnosticStatus.WARN
        memory_status.message = 'Normal' if memory_status.level == DiagnosticStatus.OK else 'High Memory Usage'
        memory_status.values.append(KeyValue(key='Memory %', value=str(mem_info.percent)))

        # Voltage Reading from Raspberry Pi
        voltage_status = DiagnosticStatus()
        voltage_status.name = 'Raspberry Pi Core Voltage'
        voltage = self.read_voltage()
        voltage_status.level = DiagnosticStatus.OK if voltage >= 4.8 else DiagnosticStatus.WARN
        voltage_status.message = 'Normal' if voltage_status.level == DiagnosticStatus.OK else 'Low Voltage'
        voltage_status.values.append(KeyValue(key='Voltage (V)', value=f'{voltage:.2f}'))

        # Publish Data
        diag_msg.status.extend([cpu_status, memory_status, voltage_status])
        self.publisher_.publish(diag_msg)
        self.get_logger().info('Diagnostics Published')


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
