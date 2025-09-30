# rokeycamp4-teamproject4-digitaltwin-turtlebot3
로키 부트캠프 4기 협동3 프로젝트 결과물 입니다

# 코드 작성 내용

우선 각 코드 별로 어떤 ws 폴더 상의 어디에 존재하는지 폴더 구조를 그대로 담아 수정한 부분만 기입했습니다.

터틀봇3 waffle 모델에서 동작한 코드이며.
각 코드는 첨부된 폴더 구조대로 구상하면 동작이 가능합니다.

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
