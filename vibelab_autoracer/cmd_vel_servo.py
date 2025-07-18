import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA

class RCCarController(Node):
    def __init__(self):
        super().__init__('rc_car_controller')

        # I2C 및 PCA9685 초기화
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # 50Hz 서보 PWM

        # 채널 설정
        self.throttle_channel = 0  # 전후진
        self.steering_channel = 1  # 조향

        # 서보 PWM 설정 (tick 기준: 0~65535)
        self.throttle_neutral = 1500  # us (중립)
        self.throttle_forward = 2000
        self.throttle_backward = 1000

        self.steering_left = 1000
        self.steering_center = 1500
        self.steering_right = 2000

        # 구독 시작
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear_x = max(min(msg.linear.x, 1.0), -1.0)     # -1.0 ~ 1.0
        angular_z = max(min(msg.angular.z, 1.0), -1.0)   # -1.0 ~ 1.0

        # 전후진 PWM 계산
        if linear_x >= 0:
            throttle_us = self.throttle_neutral + (self.throttle_forward - self.throttle_neutral) * linear_x
        else:
            throttle_us = self.throttle_neutral + (self.throttle_neutral - self.throttle_backward) * linear_x

        # 조향 PWM 계산
        steering_us = self.steering_center
        if angular_z > 0:
            steering_us = self.steering_center - (self.steering_right - self.steering_center) * angular_z
        elif angular_z < 0:
            steering_us = self.steering_center + (self.steering_left - self.steering_center) * angular_z

        self.get_logger().info(f'Throttle: {int(throttle_us)} us, Steering: {int(steering_us)} us')

        # PWM 출력
        self.pca.channels[self.throttle_channel].duty_cycle = self._pulse_to_duty(throttle_us)
        self.pca.channels[self.steering_channel].duty_cycle = self._pulse_to_duty(steering_us)

    def _pulse_to_duty(self, pulse_us):
        return int(pulse_us / 20000 * 65535)  # 20ms 주기 기준

def main(args=None):
    rclpy.init(args=args)
    node = RCCarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

