#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import cv2
import numpy as np
import time

class GstCamPublisher(Node):
    def __init__(self):
        super().__init__('gstcam_pub_node')
        
        # CompressedImage 퍼블리셔 생성
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/image_raw/compressed',
            10
        )
        
        # 카메라 초기화
        self.cap = None
        self.init_camera()
        
        if self.cap is not None:
            # 타이머로 주기적 발행 (30 FPS)
            self.timer = self.create_timer(1.0/30.0, self.publish_image)
            self.get_logger().info('✅ gstcam_publisher 시작됨 - 30 FPS로 이미지 발행')
        else:
            self.get_logger().error('❌ 카메라 초기화 실패 - 퍼블리셔 중단')
    
    def init_camera(self):
        """카메라 초기화 - CSI 카메라 우선, USB 백업"""
        
        # CSI 카메라 파이프라인들 (젯슨 전용)
        csi_pipelines = [
            ("CSI 저해상도", 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'),
            ("CSI 중해상도", 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'),
            ("CSI 고해상도", 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=20/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink')
        ]
        
        # CSI 카메라 시도
        for name, pipeline in csi_pipelines:
            if self.try_camera_pipeline(name, pipeline, is_gstreamer=True):
                return
        
        # USB 카메라 백업 시도
        for i in range(3):
            if self.try_usb_camera(i):
                return
        
        # 모든 카메라 실패
        self.get_logger().error('❌ 모든 카메라 연결 실패')
        self.get_logger().info('🔧 해결 방법:')
        self.get_logger().info('  1. CSI 카메라 리본 케이블 연결 확인')
        self.get_logger().info('  2. sudo systemctl restart nvargus-daemon')
        self.get_logger().info('  3. dmesg | grep -i camera 로 에러 확인')
        self.cap = None
    
    def try_camera_pipeline(self, name, pipeline, is_gstreamer=True):
        """카메라 파이프라인 시도"""
        try:
            self.get_logger().info(f'🔍 {name} 카메라 시도 중...')
            
            if is_gstreamer:
                self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            else:
                self.cap = cv2.VideoCapture(pipeline)
            
            if not self.cap.isOpened():
                self.get_logger().warn(f'❌ {name} 카메라 열기 실패')
                return False
            
            # 초기화 시간
            self.get_logger().info(f'⏳ {name} 카메라 초기화 중... (3초 대기)')
            time.sleep(3)
            
            # 버퍼 비우기
            for _ in range(5):
                self.cap.read()
            
            # 실제 프레임 테스트
            for attempt in range(5):
                ret, frame = self.cap.read()
                if ret and frame is not None and np.any(frame):
                    self.get_logger().info(f'✅ {name} 카메라 연결 성공!')
                    self.get_logger().info(f'📐 해상도: {frame.shape}')
                    self.get_logger().info(f'🎯 프레임 체크: min={frame.min()}, max={frame.max()}, 평균={frame.mean():.2f}')
                    return True
                else:
                    self.get_logger().warn(f'⚠️  {name} - 프레임 읽기 시도 {attempt+1}/5')
                    time.sleep(1)
            
            # 유효한 프레임 없음
            self.cap.release()
            self.get_logger().warn(f'❌ {name} 카메라 열렸지만 유효한 프레임 없음')
            return False
            
        except Exception as e:
            self.get_logger().warn(f'❌ {name} 카메라 연결 실패: {e}')
            if self.cap:
                self.cap.release()
            return False
    
    def try_usb_camera(self, device_id):
        """USB 카메라 시도"""
        try:
            self.get_logger().info(f'🔍 USB 카메라 {device_id}번 시도 중...')
            self.cap = cv2.VideoCapture(device_id)
            
            if not self.cap.isOpened():
                self.get_logger().warn(f'❌ USB 카메라 {device_id}번 열기 실패')
                return False
            
            # USB 카메라 설정
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 지연 감소
            
            time.sleep(2)  # 초기화 시간
            
            # 프레임 테스트
            for attempt in range(3):
                ret, frame = self.cap.read()
                if ret and frame is not None and np.any(frame):
                    self.get_logger().info(f'✅ USB 카메라 {device_id}번 연결 성공!')
                    self.get_logger().info(f'📐 해상도: {frame.shape}')
                    return True
                else:
                    self.get_logger().warn(f'⚠️  USB 카메라 {device_id}번 - 프레임 읽기 시도 {attempt+1}/3')
                    time.sleep(1)
            
            self.cap.release()
            self.get_logger().warn(f'❌ USB 카메라 {device_id}번 유효한 프레임 없음')
            return False
            
        except Exception as e:
            self.get_logger().warn(f'❌ USB 카메라 {device_id}번 연결 실패: {e}')
            if self.cap:
                self.cap.release()
            return False
    
    def publish_image(self):
        """이미지 읽기 및 발행"""
        if self.cap is None:
            return
        
        try:
            ret, frame = self.cap.read()
            if ret and frame is not None and np.any(frame):
                # CompressedImage 메시지 생성
                msg = CompressedImage()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                msg.format = 'jpeg'
                
                # JPEG로 압축
                encode_param = [cv2.IMWRITE_JPEG_QUALITY, 90]
                _, encoded_image = cv2.imencode('.jpg', frame, encode_param)
                msg.data = encoded_image.tobytes()
                
                # 퍼블리시
                self.image_publisher.publish(msg)
                
            else:
                self.get_logger().warn('Failed to capture frame')
                # 연속 실패 시 재연결 시도 (추가 가능)
                
        except Exception as e:
            self.get_logger().error(f'이미지 발행 에러: {e}')
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gstcam_publisher = GstCamPublisher()
        rclpy.spin(gstcam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gstcam_publisher' in locals():
            gstcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
