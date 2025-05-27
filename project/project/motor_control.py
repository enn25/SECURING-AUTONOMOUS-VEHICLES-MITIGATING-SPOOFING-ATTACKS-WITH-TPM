import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# Motor Pin Setup
LEFT_MOTOR_PWM = 13
RIGHT_MOTOR_PWM = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR_PWM, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_PWM, GPIO.OUT)

left_pwm = GPIO.PWM(LEFT_MOTOR_PWM, 1000)
right_pwm = GPIO.PWM(RIGHT_MOTOR_PWM, 1000)
left_pwm.start(0)
right_pwm.start(0)

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Simple differential drive logic
        left_speed = linear_velocity - angular_velocity
        right_speed = linear_velocity + angular_velocity

        left_pwm.ChangeDutyCycle(max(0, min(100, left_speed * 100)))
        right_pwm.ChangeDutyCycle(max(0, min(100, right_speed * 100)))

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
