import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class ScanUnitFixer(Node):
    def __init__(self):
        super().__init__('scan_unit_fixer')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, 'scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan2', 10)
        self.get_logger().info('ScanUnitFixer')

    def callback(self, msg):
        self.get_logger().info('callback')
        msg.ranges = [r / 1000.0 for r in msg.ranges]  # mm → m
        self.get_logger().info(f"Received scan with {len(msg.ranges)} ranges")
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanUnitFixer()
    rclpy.spin(node)
    rclpy.shutdown()

