import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np

qos_profile = QoSProfile(depth=10)
qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

class Autoracer(Node):
    def __init__(self):
        super().__init__('Autoracer')
        self.img_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        self.img_sub  # prevent unused variable warning

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',            # 구독할 토픽 이름 (사용하는 토픽에 맞게 변경)
            self.lidar_callback,
            qos_profile                  # QoS depth
        )
        self.lidar_sub  # prevent unused variable warning

    def image_callback(self, msg):
        #self.get_logger().info('image_callback')
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #cv2.imshow("Compressed Image", image)
        #cv2.waitKey(1)

    def lidar_callback(self, msg):
        # LaserScan 메시지 수신 시 실행되는 콜백 함수
        #self.get_logger().info(f"lidar_callback {len(msg.ranges)}")
        total_points = len(msg.ranges)
    
        # 가운데 10개 인덱스 계산
        center = total_points // 2
        half_width = 10 // 2
    
        start_idx = max(0, center - half_width)
        end_idx = min(total_points, center + half_width)
    
        center_ranges = msg.ranges[start_idx:end_idx]
        # 소수점 3자리 포맷팅
        formatted_ranges = [f"{r:.3f}" for r in center_ranges]

        self.get_logger().info(f"Center 10 ranges (3 decimals): {formatted_ranges}")
    

def main(args=None):
    rclpy.init(args=args)
    node = Autoracer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

