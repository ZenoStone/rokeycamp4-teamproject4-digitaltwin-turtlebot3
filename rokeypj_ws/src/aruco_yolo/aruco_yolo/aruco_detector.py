#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArUco detector node (CompressedImage -> detected_markers / aruco/distance)
- 카메라 compressed 이미지를 구독
- 마커가 있으면 가장 가까운 마커의 거리(/aruco/distance)와 MarkerArray 퍼블리시
- :전구: 마커가 없어도 빈 MarkerArray()를 퍼블리시 → rqt에서 항상 토픽이 보임
"""
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from aruco_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
from cv_bridge import CvBridge

def detect_markers(image, camera_matrix, dist_coeffs, marker_size):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(image)
    detect_data = []
    if ids is not None:
        # cv2.aruco.drawDetectedMarkers(image, corners, ids) # 시각화가 필요 없으므로 주석 처리 가능
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        if rvecs is not None and tvecs is not None:
            for rvec, tvec, marker_id in zip(rvecs, tvecs, ids):
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw, pitch, roll = rotationMatrixToEulerAngles(rot_mat)
                marker_pos = np.dot(-rot_mat.T, tvec).flatten()
                distance = np.linalg.norm(tvec)
                detect_data.append([int(marker_id), marker_pos, (yaw, pitch, roll), float(distance)])
    return image, detect_data
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array(
        [
            [-marker_size / 2,  marker_size / 2, 0],
            [ marker_size / 2,  marker_size / 2, 0],
            [ marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0],
        ],
        dtype=np.float32,
    )
    rvecs, tvecs = [], []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs, []
def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)
def load_camera_parameters(yaml_file):
    package_share_directory = get_package_share_directory('aruco_yolo')
    calibration_file = os.path.join(package_share_directory, 'config', yaml_file)
    with open(calibration_file, 'r') as f:
        data = yaml.safe_load(f)
        camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float32)
    return camera_matrix, dist_coeffs

class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.get_logger().info('ArucoMarkerDetector(Node)')
        # 입력: CompressedImage
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',          # 필요시 --ros-args -r 로 리맵
            self.listener_callback,
            10
        )
        # 출력: 마커 배열(항상 퍼블리시), 가장 가까운 마커 거리
        self.marker_publisher = self.create_publisher(MarkerArray, 'detected_markers', 10)
        self.distance_publisher_ = self.create_publisher(Float32, '/aruco/distance', 10)
        self.bridge = CvBridge()
        self.marker_size = 0.04  # meters (arg로 덮어씀)
        # 카메라 캘리브레이션 로드
        self.camera_matrix, self.dist_coeffs = load_camera_parameters('calibration_params.yaml')
        # # 미리보기 창 (제거됨)
        # cv2.namedWindow('Detected Markers', cv2.WINDOW_NORMAL)
    def listener_callback(self, msg):
        # compressed -> BGR
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # 마커 검출
        frame, detect_data = detect_markers(frame, self.camera_matrix, self.dist_coeffs, self.marker_size)
        # 항상 퍼블리시할 MarkerArray 준비
        marker_array_msg = MarkerArray()
        if len(detect_data) == 0:
            # :작은_파란색_다이아몬드: 마커가 없어도 빈 배열 퍼블리시 → rqt에서 토픽이 항상 보임
            self.marker_publisher.publish(marker_array_msg)
            self.get_logger().debug('No markers -> publish empty MarkerArray()')
        else:
            # 가장 가까운 마커
            closest_marker = min(detect_data, key=lambda x: x[3])
            # 거리 퍼블리시
            msg_distance = Float32()
            msg_distance.data = closest_marker[3]
            self.distance_publisher_.publish(msg_distance)
            self.get_logger().info(f'Publishing distance: {msg_distance.data:.2f} m')
            # # 텍스트 오버레이 (제거해도 무방)
            # overlay = f"x:{closest_marker[1][0]:.2f}, z:{closest_marker[1][2]:.2f}, d:{closest_marker[3]:.2f}m"
            # cv2.putText(frame, overlay, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            #             (0, 255, 0), 2, cv2.LINE_AA)
            # (예전 로직 유지) 가장 가까운 마커만 MarkerArray에 포함
            for m_id, pos, euler, _dist in detect_data:
                if int(closest_marker[0]) != int(m_id):
                    continue
                marker_msg = Marker()
                marker_msg.id = int(m_id)
                marker_msg.pose.pose.position.x = float(pos[0])
                marker_msg.pose.pose.position.y = float(pos[1])
                marker_msg.pose.pose.position.z = float(pos[2])
                marker_msg.pose.pose.orientation.x = float(euler[2])  # (원본 코드 유지)
                marker_msg.pose.pose.orientation.y = float(euler[1])
                marker_msg.pose.pose.orientation.z = float(euler[0])
                marker_array_msg.markers.append(marker_msg)
            self.marker_publisher.publish(marker_array_msg)
            self.get_logger().info(f'Publishing MarkerArray with {len(marker_array_msg.markers)} marker(s)')
        # # 미리보기 (제거됨)
        # cv2.imshow('Detected Markers', frame)
        # cv2.resizeWindow('Detected Markers', 320, 240)
        # cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Detect ArUco markers.')
    parser.add_argument('--marker_size', type=float, default=0.04,
                        help='Size of the ArUco markers in meters.')
    argv = rclpy.utilities.remove_ros_args()  # argparse와 ros-args 충돌 방지
    parsed, _unknown = parser.parse_known_args(argv)
    node = ArucoMarkerDetector()
    node.marker_size = parsed.marker_size
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()














