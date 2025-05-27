import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor, PWMOutputDevice
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Motor and PWM Setup
        self.motor_left = Motor(forward=17, backward=18)
        self.motor_right = Motor(forward=22, backward=23)
        self.speed_left = PWMOutputDevice(13)
        self.speed_right = PWMOutputDevice(12)

        # Track last received command time
        self.last_command_time = time.time()
        self.timeout = 0.5  # 500 ms timeout to stop motors if no input

        # Create a timer to check for timeout
        self.timer = self.create_timer(0.1, self.check_timeout)

    def cmd_vel_callback(self, msg):
        # Get linear and angular values from the Twist message
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Calculate motor speeds for differential drive
        left_speed = linear_speed - angular_speed
        right_speed = linear_speed + angular_speed

        # Clamp speed values to the valid range (0 to 1)
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        # Control left motor
        if left_speed > 0:
            self.motor_left.forward()
        elif left_speed < 0:
            self.motor_left.backward()
        else:
            self.motor_left.stop()

        # Control right motor
        if right_speed > 0:
            self.motor_right.forward()
        elif right_speed < 0:
            self.motor_right.backward()
        else:
            self.motor_right.stop()

        # Set PWM values for speed
        self.speed_left.value = abs(left_speed)
        self.speed_right.value = abs(right_speed)

        # Update the last command time
        self.last_command_time = time.time()

    def check_timeout(self):
        # Stop motors if no command received within the timeout period
        if time.time() - self.last_command_time > self.timeout:
            self.motor_left.stop()
            self.motor_right.stop()
            self.speed_left.value = 0
            self.speed_right.value = 0


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
