import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CompressedCameraPublisher(Node):
    def __init__(self):
        super().__init__('compressed_camera_publisher')
        self.publisher = self.create_publisher(CompressedImage, 'image_raw/compressed', 1)

        gst_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=5/1 ! "
            "nvjpegenc ! "
            "appsink"
        )
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 약 10fps

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        ret, jpeg = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()
        self.publisher.publish(msg)
        #self.get_logger().info("Published compressed frame")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


