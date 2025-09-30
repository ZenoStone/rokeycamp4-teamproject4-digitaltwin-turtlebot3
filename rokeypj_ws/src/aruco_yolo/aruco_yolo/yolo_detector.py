#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Traffic Light Color Detector using ROI and Color Segmentation.
- 오른쪽 ROI에서 red/green/yellow 검출
- data("traffic_light/color") 퍼블리시
- :작은_파란색_다이아몬드: red=정지, yellow=서행, green=재가속 (최소 수정으로 제어 신호 퍼블리시)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time  # (선택) 디바운스에 사용 가능
class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')
        self.get_logger().info('Color Detector node has been started.')
        # ── 구독: 카메라 영상
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.listener_callback, 10
        )
        # ── 퍼블리셔: 색상 문자열, 디버그 이미지
        self.color_publisher_ = self.create_publisher(String, 'traffic_light/color', 10)
        self.img_publisher_ = self.create_publisher(CompressedImage, 'traffic_light/image', 10)
        # ── :작은_파란색_다이아몬드: 추가 퍼블리셔: 속도 제어
        # 게이트 방식: /control/max_vel 로 최대 속도 지시
        self.maxvel_pub_ = self.create_publisher(Float64, '/control/max_vel', 10)
        # 즉시 정지 신호(필요 시): /cmd_vel 로 0 게시 (게이트가 있어도 안전하게)
        self.cmdvel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # ── 파라미터(원하면 launch에서 -p 로 조절)
        self.declare_parameter('go_speed', 0.20)     # 초록 불: 평속
        self.declare_parameter('slow_speed', 0.08)   # 노랑 불: 서행 속도
        self.declare_parameter('stop_on_red', True)  # 빨강 불: 완전정지 수행 여부
        self.go_speed = float(self.get_parameter('go_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.stop_on_red = bool(self.get_parameter('stop_on_red').value)
        # ── HSV 범위
        self.lower_green = np.array([40, 80, 80])
        self.upper_green = np.array([80, 255, 255])
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        # (선택) 색 전환 디바운스를 위한 상태
        self.last_applied_color = None
        self.last_apply_time = 0.0
        self.min_hold_sec = 0.2  # 너무 빠른 깜빡임 방지

    def listener_callback(self, msg):

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")
            return
        
        height, width, _ = cv_image.shape
        # ── ROI: 오른쪽 절반
        roi_x_start = width // 2
        roi = cv_image[0:height, roi_x_start:width]
        # ── HSV 마스크
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_green  = cv2.inRange(hsv_roi, self.lower_green, self.upper_green)
        mask_red1   = cv2.inRange(hsv_roi, self.lower_red1,  self.upper_red1)
        mask_red2   = cv2.inRange(hsv_roi, self.lower_red2,  self.upper_red2)
        mask_red    = cv2.add(mask_red1, mask_red2)
        mask_yellow = cv2.inRange(hsv_roi, self.lower_yellow, self.upper_yellow)
        # ── 픽셀 카운트
        green_pixels  = cv2.countNonZero(mask_green)
        red_pixels    = cv2.countNonZero(mask_red)
        yellow_pixels = cv2.countNonZero(mask_yellow)
        # ── 색 판단
        detected_color = "none"
        detection_threshold = 100

        if green_pixels > detection_threshold and green_pixels >= red_pixels and green_pixels >= yellow_pixels:
            detected_color = "green"
        elif red_pixels > detection_threshold and red_pixels >= green_pixels and red_pixels >= yellow_pixels:
            detected_color = "red"
        elif yellow_pixels > detection_threshold and yellow_pixels >= green_pixels and yellow_pixels >= red_pixels:
            detected_color = "yellow"

        # ── 결과 퍼블리시 (문자열)
        color_msg = String()
        color_msg.data = detected_color
        self.color_publisher_.publish(color_msg)
        # ── :작은_파란색_다이아몬드: 제어 로직 적용 (최소 수정 포인트)
        self.apply_traffic_action(detected_color)
        # ── 디버깅 시각화
        cv2.rectangle(cv_image, (roi_x_start, 0), (width, height), (255, 255, 0), 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image, f"Detected: {detected_color}", (roi_x_start, 30),
                    font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        _, compressed_image = cv2.imencode('.jpg', cv_image, encode_param)
        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.format = "jpeg"
        img_msg.data = compressed_image.tobytes()
        self.img_publisher_.publish(img_msg)

    # ─────────────────────────────────────────────────────────────
    # :작은_파란색_다이아몬드: 빨강=정지 / 노랑=서행 / 초록=재출발 (게이트+즉시정지)
    # ─────────────────────────────────────────────────────────────
    def apply_traffic_action(self, color: str):
        now = time.time()
        # 디바운스: 너무 자주 바뀌면 무시
        if self.last_applied_color == color and (now - self.last_apply_time) < self.min_hold_sec:
            return
        if color == 'red':
            # 최대 속도 0 (게이트)
            self.maxvel_pub_.publish(Float64(data=0.0))
            if self.stop_on_red:
                # 즉시 정지 토픽도 한 번 쏴줌 (더 빠른 브레이크)
                tw = Twist()
                tw.linear.x = 0.0
                tw.angular.z = 0.0
                self.cmdvel_pub_.publish(tw)
            self.get_logger().info('Traffic: RED -> STOP')
        elif color == 'yellow':
            # 서행
            self.maxvel_pub_.publish(Float64(data=self.slow_speed))
            self.get_logger().info(f'Traffic: YELLOW -> SLOW ({self.slow_speed:.2f} m/s)')
        elif color == 'green':
            # 재가속
            self.maxvel_pub_.publish(Float64(data=self.go_speed))
            self.get_logger().info(f'Traffic: GREEN -> GO ({self.go_speed:.2f} m/s)')
        else:
            # 'none'일 때는 기존 상태 유지 (아무 것도 안 함)
            return
        self.last_applied_color = color
        self.last_apply_time = now
        
def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()