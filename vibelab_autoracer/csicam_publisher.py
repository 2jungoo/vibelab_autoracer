import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CSIcamPublisher(Node):
    WIDTH = 1280
    HEIGHT = 720
    FPS = 30
    SRC = 'nvarguscamerasrc bufapi-version=1 ! video/x-raw(memory:NVMM), width='+WIDTH+', height='+HEIGHT+', format=NV12, framerate='+FPS+'/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false'


    def __init__(self):
        super().__init__('csicam_publisher')
        self.get_logger().info('csicam_publisher initialized.')
        self.publisher_ = self.create_publisher(Image, 'csicam/image_raw', 1)
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(CSIcamPublisher.SRC, cv2.CAP_GSTREAMER)
        # self.cap = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3840, height=2160, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink', cv2.CAP_GSTREAMER)

        if self.cap.isOpened():
            self.timer = self.create_timer(1.0/CSIcamPublisher.FPS, self.timer_callback)  # 10 Hz
        else:
            self.get_logger().error('Failed to open camera.')


    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            cv2.imshow("Camera", frame)
            image_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)
        else:
            self.get_logger().warn('Failed to capture frame from camera.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CSIcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
