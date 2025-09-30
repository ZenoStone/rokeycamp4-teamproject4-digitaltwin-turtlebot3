#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 파라미터 파일의 경로를 정확히 지정해야 합니다.
    # 만약 'turtlebot3_autorace_detect' 패키지에 있다면 해당 패키지 이름을 사용하세요.
    detect_param = os.path.join(
        get_package_share_directory('aruco_yolo'), # 파라미터 파일이 있는 패키지
        'param', 'lane', 'lane.yaml' # 실제 파라미터 파일 경로
    )

    return LaunchDescription([
        # 카메라 노드는 별도로 실행한다고 가정합니다.

        # 2. ArUco 마커를 보는 눈
        Node(
            package='aruco_yolo',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
        ),

        # 3. 신호등 색을 보는 눈
        Node(
            package='aruco_yolo',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
        ),

        # <<< 수정: 차선 감지 노드 주석 해제 >>>
        # 4. 차선을 보는 눈 (이제 활성화됨!)
        Node(
            package='aruco_yolo', # detect_lane.py가 있는 패키지
            executable='detect_lane',
            name='detect_lane',
            parameters=[detect_param],
            output='screen',
            remappings=[
                # 이 노드는 압축된 이미지 토픽이 필요합니다.
                # 카메라가 '/camera/image_raw/compressed' 토픽을 발행하는지 확인하세요.
                

                ('/detect/image_input', '/camera/image_projected'),
                ('/detect/image_input/compressed', '/camera/image_projected/compressed'),
                ('/detect/image_output', '/detect/image_lane'),
                ('/detect/image_output/compressed', '/detect/image_lane/compressed'),
                ('/detect/image_output_sub1', '/detect/image_white_lane_marker'),
                ('/detect/image_output_sub1/compressed', '/detect/image_white_lane_marker/compressed'),
                ('/detect/image_output_sub2', '/detect/image_yellow_lane_marker'),
                ('/detect/image_output_sub2/compressed', '/detect/image_yellow_lane_marker/compressed')
            ]
        ),
        
        # 5. 차단기를 보는 눈
        Node(
            package='aruco_yolo',
            executable='detect_level_crossing',
            name='level_crossing_sensor',
            output='screen'
        ),

        # 6. 유일한 두뇌 (모든 운전은 여기서!)
        Node(
            package='aruco_yolo',
            executable='mission_controller',
            name='mission_controller',
            output='screen',
        ),
    ])