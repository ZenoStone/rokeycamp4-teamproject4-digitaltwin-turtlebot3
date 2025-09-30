#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission Controller Node (The Brain) - 차선 제어 통합 버전

- 모든 미션 제어와 차선 주행 로직 통합.
- 차단기 마커를 발견하면 서행으로 접근.
- 닫힌 차단기를 감지하면 즉시 정지.
- PD 제어기를 사용하여 차선 주행을 직접 수행.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64  # <<< 수정: Int16를 Float64로 변경
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
import time

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.get_logger().info('차선 제어 로직이 통합된 미션 컨트롤러가 시작되었습니다.')

        # --- 로봇 상태 (State Machine) ---
        self.robot_state = 'LANE_FOLLOWING'
        # 다른 상태: 'APPROACHING_GATE', 'WAITING_FOR_GREEN', 'WAITING_FOR_GATE', 'CROSSING', 'PASSING_GATE'

        # --- 미션 파라미터 ---
        self.stop_marker_id = 0
        self.gate_marker_id = 1
        self.crossing_duration = 4.0
        self.gate_passing_duration = 3.0

        # --- 차선 주행 및 속도 파라미터 ---
        # <<< 추가: 차선 주행 제어를 위한 파라미터 >>>
        self.declare_parameter('target_lane_center', 500.0) # detect_lane이 보내는 이미지의 목표 중심값
        self.declare_parameter('kp', 0.0025) # 비례 이득
        self.declare_parameter('kd', 0.007)  # 미분 이득
        self.target_lane_center = self.get_parameter('target_lane_center').get_parameter_value().double_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        
        self.drive_speed = 0.08         # 일반 차선 주행 최대 속도
        self.gate_approach_speed = 0.04 # 차단기 접근 시 서행 속도
        self.crossing_speed = 0.10      # 교차로 횡단 속도

        # --- 내부 변수 ---
        self.lane_center_pos = self.target_lane_center # <<< 수정: /detect/lane 토픽에서 차선 중앙값을 저장
        self.last_error = 0.0                          # <<< 추가: PD 제어기용 이전 오차값
        self.crossing_start_time = 0
        self.gate_passing_start_time = 0
        self.level_crossing_status = 'UNKNOWN'

        # --- 구독자 (Subscriber) ---
        self.traffic_light_sub = self.create_subscription(
            String, '/traffic_light/color', self.traffic_light_callback, 10)
        self.aruco_marker_sub = self.create_subscription(
            MarkerArray, 'detected_markers', self.aruco_marker_callback, 10)
        # <<< 수정: 발행 노드와 타입을 맞추기 위해 Float64로 변경 >>>
        self.lane_sub = self.create_subscription(
            Float64, '/detect/lane', self.lane_callback, 10)
        self.level_crossing_sub = self.create_subscription(
            String, '/detect/level_crossing_status', self.level_crossing_callback, 10)

        # --- 발행자 (Publisher) ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def traffic_light_callback(self, msg):
        if self.robot_state != 'WAITING_FOR_GREEN': return
        if msg.data == 'green':
            self.get_logger().info('✅ 초록불 감지! 횡단을 시작합니다.')
            self.robot_state = 'CROSSING'
            self.crossing_start_time = time.time()

    def aruco_marker_callback(self, msg):
        if self.robot_state not in ['LANE_FOLLOWING', 'APPROACHING_GATE']: return
        for marker in msg.markers:
            if marker.id == self.stop_marker_id and self.robot_state == 'LANE_FOLLOWING':
                self.get_logger().info(f'🛑 정지 마커 감지! 초록불을 대기합니다.')
                self.robot_state = 'WAITING_FOR_GREEN'
                break
            elif marker.id == self.gate_marker_id and self.robot_state == 'LANE_FOLLOWING':
                self.get_logger().info(f'🚧 차단기 마커 감지! 천천히 접근합니다.')
                self.robot_state = 'APPROACHING_GATE'
                break

    # <<< 수정: 차선 감지 노드에서 받은 float 값을 저장하는 콜백 함수 >>>
    def lane_callback(self, msg):
        self.lane_center_pos = msg.data

    def level_crossing_callback(self, msg):
        self.level_crossing_status = msg.data
        if self.robot_state == 'APPROACHING_GATE' and self.level_crossing_status == 'DETECTED_CLOSE':
            self.get_logger().info('🔴 닫힌 차단기 감지! 즉시 정지합니다.')
            self.robot_state = 'WAITING_FOR_GATE'
        elif self.robot_state == 'WAITING_FOR_GATE' and self.level_crossing_status == 'OPEN':
            self.get_logger().info('🟢 차단기가 열렸습니다! 통과를 시작합니다.')
            self.robot_state = 'PASSING_GATE'
            self.gate_passing_start_time = time.time()

    def control_loop(self):
        move_cmd = Twist()
        
        # <<< 수정: PD 제어기를 이용한 차선 주행 로직 통합 >>>
        if self.robot_state == 'LANE_FOLLOWING' or self.robot_state == 'APPROACHING_GATE':
            error = self.lane_center_pos - self.target_lane_center
            
            # 조향을 위한 PD 제어
            angular_z = self.kp * error + self.kd * (error - self.last_error)
            self.last_error = error
            move_cmd.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            
            # 상태에 따라 선속도 설정
            if self.robot_state == 'LANE_FOLLOWING':
                # 오차가 클수록(코너링) 속도를 줄임
                speed_factor = max(1 - abs(error) / self.target_lane_center, 0) ** 2.2
                move_cmd.linear.x = self.drive_speed * speed_factor
            else: # 'APPROACHING_GATE' 상태
                move_cmd.linear.x = self.gate_approach_speed

        elif self.robot_state in ['WAITING_FOR_GREEN', 'WAITING_FOR_GATE']:
            move_cmd.linear.x, move_cmd.angular.z = 0.0, 0.0
        
        elif self.robot_state == 'CROSSING':
            if time.time() - self.crossing_start_time < self.crossing_duration:
                move_cmd.linear.x, move_cmd.angular.z = self.crossing_speed, 0.0
            else:
                self.get_logger().info('🏁 횡단 완료. 차선 주행을 재개합니다.')
                self.robot_state = 'LANE_FOLLOWING'

        elif self.robot_state == 'PASSING_GATE':
            if time.time() - self.gate_passing_start_time < self.gate_passing_duration:
                move_cmd.linear.x, move_cmd.angular.z = self.drive_speed, 0.0
            else:
                self.get_logger().info('🏁 차단기 통과 완료. 차선 주행을 재개합니다.')
                self.robot_state = 'LANE_FOLLOWING'
        
        self.cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    mission_controller = MissionController()
    rclpy.spin(mission_controller)
    mission_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()