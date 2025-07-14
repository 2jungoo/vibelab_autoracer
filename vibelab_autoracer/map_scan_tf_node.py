# oar_rc/map_scan_tf_node.py
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations


class MapScanTFNode(Node):
    def __init__(self):
        super().__init__('map_scan_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        # map -> base_link
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'base_link'
        t1.transform.translation.x = 1.0  # 예시 위치
        t1.transform.translation.y = 2.0
        t1.transform.translation.z = 0.0
        q1 = tf_transformations.quaternion_from_euler(0, 0, math.radians(45))
        t1.transform.rotation.x = q1[0]
        t1.transform.rotation.y = q1[1]
        t1.transform.rotation.z = q1[2]
        t1.transform.rotation.w = q1[3]

        # base_link -> scan
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'laser'
        t2.transform.translation.x = 0.2
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.1
        q2 = tf_transformations.quaternion_from_euler(0, 0, 0)
        t2.transform.rotation.x = q2[0]
        t2.transform.rotation.y = q2[1]
        t2.transform.rotation.z = q2[2]
        t2.transform.rotation.w = q2[3]

        self.br.sendTransform(t1)
        self.br.sendTransform(t2)


def main(args=None):
    rclpy.init(args=args)
    node = MapScanTFNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

