#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Level Crossing Detector (Sensor Role Only)
- 로직: Dual-Red HSV → ROI → 컨투어 → 회전사각형으로 막대 상태 판정
- 출력: /detect/level_crossing_status (String), 디버그 이미지
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge

class LevelCrossingSensor(Node):
    def __init__(self):
        super().__init__('level_crossing_sensor')
        self.get_logger().info('Level Crossing Sensor node has been started.')

        # --- 발행자 (Publisher) ---
        # 1. 차단기 상태 발행 (가장 중요!)
        self.status_publisher = self.create_publisher(String, '/detect/level_crossing_status', 10)
        # 2. 디버깅용 이미지 발행
        self.img_publisher = self.create_publisher(CompressedImage, '/detect/image_level_crossing/compressed', 10)

        # --- 구독자 (Subscriber) ---
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        
        # --- 파라미터 (시각 처리용) ---
        self.declare_parameter('red1.l', [0, 100, 100])
        self.declare_parameter('red1.h', [10,  255, 255])
        self.declare_parameter('red2.l', [170, 100, 100])
        self.declare_parameter('red2.h', [179, 255, 255])
        self.declare_parameter('roi_ratio_top',    0.10)
        self.declare_parameter('roi_ratio_bottom', 0.15)
        self.declare_parameter('roi_ratio_left',   0.05)
        self.declare_parameter('roi_ratio_right',  0.05)
        self.declare_parameter('min_bar_area',     400.0)
        self.declare_parameter('min_bar_length',   60.0)
        self.declare_parameter('min_aspect_ratio', 2.5)
        self.declare_parameter('angle_close_deg',  20.0)
        self.declare_parameter('angle_open_deg',   20.0)
        self.declare_parameter('angle_open_lo_deg',  35.0)
        self.declare_parameter('tip_raise_frac',   0.05)

        self.declare_parameter('closed_aspect_ratio', 4.0) # ⭐ 추가: 닫힘 판정을 위한 가로/세로 비율


        # 파라미터 값 가져오기
        self.red1_l = np.array(self.get_parameter('red1.l').value, dtype=np.uint8)
        self.red1_h = np.array(self.get_parameter('red1.h').value, dtype=np.uint8)
        self.red2_l = np.array(self.get_parameter('red2.l').value, dtype=np.uint8)
        self.red2_h = np.array(self.get_parameter('red2.h').value, dtype=np.uint8)
        self.roi_ratio_top    = float(self.get_parameter('roi_ratio_top').value)
        self.roi_ratio_bottom = float(self.get_parameter('roi_ratio_bottom').value)
        self.roi_ratio_left   = float(self.get_parameter('roi_ratio_left').value)
        self.roi_ratio_right  = float(self.get_parameter('roi_ratio_right').value)
        self.min_bar_area     = float(self.get_parameter('min_bar_area').value)
        self.min_bar_length   = float(self.get_parameter('min_bar_length').value)
        self.min_aspect_ratio = float(self.get_parameter('min_aspect_ratio').value)
        self.angle_close_deg  = float(self.get_parameter('angle_close_deg').value)
        self.angle_open_deg   = float(self.get_parameter('angle_open_deg').value)
        self.angle_open_lo_deg  = float(self.get_parameter('angle_open_lo_deg').value)
        self.tip_raise_frac   = float(self.get_parameter('tip_raise_frac').value)
        self.closed_aspect_ratio = float(self.get_parameter('closed_aspect_ratio').value) # ⭐ 추가

    def image_callback(self, msg):
        # self.get_logger().info('✅ 1')

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")
            return
        
        # 1. 빨간색 마스크 생성
        mask = self._mask_red_led(frame)
        # 2. 마스크로부터 차단기 상태와 시각화 이미지 추론
        status, vis_img = self._infer_bar(mask, frame)
        

        # self.get_logger().info('✅ 12')

        # 3. 추론된 상태를 String 메시지로 발행
        status_msg = String()
        status_msg.data = status # status는 "OPEN", "DETECTED_CLOSE" 등의 문자열
        self.status_publisher.publish(status_msg)

        # self.get_logger().info('✅ 123')


        # 4. 디버깅용 시각화 이미지 발행
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        _, compressed_image = cv2.imencode('.jpg', vis_img, encode_param)
        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.format = "jpeg"
        img_msg.data = compressed_image.tobytes()
        self.img_publisher.publish(img_msg)

    def _mask_red_led(self, bgr):
        h, w = bgr.shape[:2]
        x1 = int(self.roi_ratio_left   * w)
        x2 = int(w - self.roi_ratio_right * w)
        y1 = int(self.roi_ratio_top    * h)
        y2 = int(h - self.roi_ratio_bottom * h)
        roi = bgr[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, self.red1_l, self.red1_h)
        m2 = cv2.inRange(hsv, self.red2_l, self.red2_h)
        mask = cv2.bitwise_or(m1, m2)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
        full = np.zeros((h, w), dtype=np.uint8)
        full[y1:y2, x1:x2] = mask
        return full

    # detect_level_crossing.py 파일의 _infer_bar 함수를 아래 내용으로 전체 교체하세요.

    def _infer_bar(self, mask, frame_bgr):
        status = 'UNKNOWN'
        vis = frame_bgr.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return 'OPEN', vis

        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < self.min_bar_area:
            return 'OPEN', vis

        # --- ⭐ 새로운 '닫힘' 판정 로직 (안정성 UP!) ---
        # 1. 회전 없는 일반 바운딩 박스를 계산합니다.
        _x, _y, w, h = cv2.boundingRect(cnt)
        
        # 2. 세로 길이가 0인 경우를 방지합니다.
        if h == 0: h = 1 

        # 3. 가로/세로 비율을 계산합니다.
        aspect_ratio_std = float(w) / h
        
        # 4. 이 비율이 설정한 임계값(e.g., 4.0)보다 크면 '닫힘'으로 확정합니다.
        if aspect_ratio_std > self.closed_aspect_ratio:
            status = 'DETECTED_CLOSE'
            # 시각화를 위해 회전 사각형도 그려줍니다.
            rect = cv2.minAreaRect(cnt)
            box = np.intp(cv2.boxPoints(rect))
            cv2.drawContours(vis, [box], 0, (0, 255, 255), 2)
            return status, vis
        
        # --- 기존의 '열림' 판정 로직 (회전 사각형 기반) ---
        # (위에서 '닫힘'이 아니라고 판명된 경우에만 실행됩니다)
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), ang = rect
        box = np.intp(cv2.boxPoints(rect))
        cv2.drawContours(vis, [box], 0, (0, 255, 255), 2)

        long_len = max(rw, rh)
        short_len = min(rw, rh)
        if short_len == 0: return 'UNKNOWN', vis
        
        aspect = float(long_len) / float(short_len)
        if long_len < self.min_bar_length or aspect < self.min_aspect_ratio:
            return 'UNKNOWN', vis
            
        angle_abs = abs(ang)
        if long_len == rh:
            angle_abs = abs(ang + 90.0)
            
        theta = np.deg2rad(ang if long_len == rw else ang + 90.0)
        ux, uy = np.cos(theta), np.sin(theta)
        p1 = np.array([cx + 0.5 * long_len * ux, cy + 0.5 * long_len * uy])
        p2 = np.array([cx - 0.5 * long_len * ux, cy - 0.5 * long_len * uy])
        
        top_y = min(p1[1], p2[1])
        bottom_y = max(p1[1], p2[1])
        h_img = frame_bgr.shape[0]
        tip_raise_ok = (bottom_y - top_y) >= (self.tip_raise_frac * h_img)

        # '열림' 또는 '애매한' 상태 판정
        if abs(angle_abs - 90.0) <= self.angle_open_deg:
            status = 'OPEN'
        elif angle_abs >= self.angle_open_lo_deg or tip_raise_ok:
            status = 'OPEN'
        else:
            status = 'DETECTED_FAR'
            
        return status, vis



def main(args=None):
    rclpy.init(args=args)
    node = LevelCrossingSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()