# rokeycamp4-teamproject4-digitaltwin-turtlebot3
로키 부트캠프 4기 협동3 프로젝트 결과물 입니다

turtlebot3 waffle 모델에 manipulator가 설치된 모델을 사용하여 구성된 프로젝트 입니다
사용한 turtlebot3는 Jetson Nano가 설치된 모델입니다

turtlebot3 waffle 모델을 사용하여 제한된 공간에서 라인 트레이싱을 통한 주행을 바탕으로
정의된 아르코 마커를 로봇의 manipulator에 달린 카메라가 감지하면 신호등 혹은 차단기 인식을 진행합니다

> 아르코 마커 ID 0 : 신호등
> 아르코 마커 ID 1 : 차단기


포함된 ros2 패키지는 turtlebot3 Jetson Nano에 파일구조를 생성한 다음에 turtlebot3에서 구동해야 합니다

본 프로젝트는 rokeypj_ws에 구성된 하나의 aruco_yolo.launch.py를 실행해서 모든 동작을 수행할 수 있습니다


# 실행 방법
1. 첨부된 rokeypj_ws와 rokeyracing_ws를 사용자의 홈 디렉토리에 배치.
2. 터미널 상에서 각 '_ws' 폴더 상에 진입 후 colcon build
3. colcon build 후 각각 '_ws' 폴더 상에서 source install/setup.bash
5. 터미널 상에서 rokeypj_ws로 이동 후 aruco_yolo.launch.py 실행
> 작성한 코드는 모두 aruco_yolo.launch.py 파일 하나로 동작 가능합니다.

# 카메라 설정

1. 사용한 로봇팔의 각도는 turtlebot3_manupulate_angle.png를 토대로 구상했습니다.
2. ~/finalPJ_code/rokeyracing_ws/src/turtlebot3_autorace_camera/calibration/extrinsic_calibration 경로에 존재하는 .yaml 파일에 c-2조 로봇 설정에 사용한 카메라 파라미터 값이 compensation.yaml, projection.yaml로 설정되어 있습니다.

## License
This project is licensed under the MIT License. See the LICENSE file for details.
