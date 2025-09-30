import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import UInt8


class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        self.get_logger().info('detect_lane start.')


        parameter_descriptor_hue = ParameterDescriptor(
            description='hue parameter range',
            integer_range=[IntegerRange(
                from_value=0,
                to_value=179,
                step=1)]
        )
        parameter_descriptor_saturation_lightness = ParameterDescriptor(
            description='saturation and lightness range',
            integer_range=[IntegerRange(
                from_value=0,
                to_value=255,
                step=1)]
        )
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detect.lane.white.hue_l', 0,
                    parameter_descriptor_hue),
                ('detect.lane.white.hue_h', 179,
                    parameter_descriptor_hue),
                ('detect.lane.white.saturation_l', 0,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.saturation_h', 70,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_l', 105,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.hue_l', 10,
                    parameter_descriptor_hue),
                ('detect.lane.yellow.hue_h', 127,
                    parameter_descriptor_hue),
                ('detect.lane.yellow.saturation_l', 70,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.saturation_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_l', 95,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('is_detection_calibration_mode', False)
            ]
        )

        self.hue_white_l = self.get_parameter(
            'detect.lane.white.hue_l').get_parameter_value().integer_value
        self.hue_white_h = self.get_parameter(
            'detect.lane.white.hue_h').get_parameter_value().integer_value
        self.saturation_white_l = self.get_parameter(
            'detect.lane.white.saturation_l').get_parameter_value().integer_value
        self.saturation_white_h = self.get_parameter(
            'detect.lane.white.saturation_h').get_parameter_value().integer_value
        self.lightness_white_l = self.get_parameter(
            'detect.lane.white.lightness_l').get_parameter_value().integer_value
        self.lightness_white_h = self.get_parameter(
            'detect.lane.white.lightness_h').get_parameter_value().integer_value

        self.hue_yellow_l = self.get_parameter(
            'detect.lane.yellow.hue_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter(
            'detect.lane.yellow.hue_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter(
            'detect.lane.yellow.saturation_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter(
            'detect.lane.yellow.saturation_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter(
            'detect.lane.yellow.lightness_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter(
            'detect.lane.yellow.lightness_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_detection_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.cbGetDetectLaneParam)

        self.sub_image_type = 'raw'
        self.pub_image_type = 'compressed'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.cbFindLane, 1
                )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self.cbFindLane, 1
                )

        if self.pub_image_type == 'compressed':
            self.pub_image_lane = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1
                )
        elif self.pub_image_type == 'raw':
            self.pub_image_lane = self.create_publisher(
                Image, '/detect/image_output', 1
                )

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 1
                    )
                self.pub_image_yellow_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub2/compressed', 1
                    )
            elif self.pub_image_type == 'raw':
                self.pub_image_white_lane = self.create_publisher(
                    Image, '/detect/image_output_sub1', 1
                    )
                self.pub_image_yellow_lane = self.create_publisher(
                    Image, '/detect/image_output_sub2', 1
                    )

        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)
        self.pub_yellow_line_reliability = self.create_publisher(
            UInt8, '/detect/yellow_line_reliability', 1
            )
        self.pub_white_line_reliability = self.create_publisher(
            UInt8, '/detect/white_line_reliability', 1
            )
        self.pub_lane_state = self.create_publisher(UInt8, '/detect/lane_state', 1)
        self.cvBridge = CvBridge()
        self.counter = 1
        self.window_width = 1000.
        self.window_height = 600.
        self.reliability_white_line = 100
        self.reliability_yellow_line = 100
        self.mov_avg_left = np.empty((0, 3))
        self.mov_avg_right = np.empty((0, 3))
        self.pub_image_roi = self.create_publisher(
            CompressedImage, '/detect/roi_image/compressed', 1
        )
        self.last_lane_offset = 280
        self.lane_offset_alpha = 0.1

        # ✨ 관련 변수들을 올바른 형태로 미리 초기화합니다.
        self.last_known_centerx = np.full(int(self.window_height), self.window_width / 2, dtype=float)
        self.left_fit = np.array([0., 0., 0.])
        self.right_fit = np.array([0., 0., 0.])
        self.left_fit_bef = np.array([0., 0., 0.])
        self.right_fit_bef = np.array([0., 0., 0.])
        self.left_fitx = None
        self.right_fitx = None

        # ✨ CLAHE 객체 초기화 ✨
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.get_logger().info('detect_lane setting end.')

    def cbGetDetectLaneParam(self, parameters):
        for param in parameters:
            if param.name == 'detect.lane.white.hue_l':
                self.hue_white_l = param.value
            elif param.name == 'detect.lane.white.hue_h':
                self.hue_white_h = param.value
            elif param.name == 'detect.lane.white.saturation_l':
                self.saturation_white_l = param.value
            elif param.name == 'detect.lane.white.saturation_h':
                self.saturation_white_h = param.value
            elif param.name == 'detect.lane.white.lightness_l':
                self.lightness_white_l = param.value
            elif param.name == 'detect.lane.white.lightness_h':
                self.lightness_white_h = param.value
            elif param.name == 'detect.lane.yellow.hue_l':
                self.hue_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.hue_h':
                self.hue_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.saturation_l':
                self.saturation_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.saturation_h':
                self.saturation_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.lightness_l':
                self.lightness_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.lightness_h':
                self.lightness_yellow_h = param.value
        return SetParametersResult(successful=True)

    def auto_white_balance(self, img):
        b, g, r = cv2.split(img)
        avg_b = np.mean(b)
        avg_g = np.mean(g)
        avg_r = np.mean(r)
        avg_gray = (avg_b + avg_g + avg_r) / 3
        if avg_b == 0 or avg_g == 0 or avg_r == 0:
            return img
        b = np.minimum(255, b * (avg_gray / avg_b)).astype(np.uint8)
        g = np.minimum(255, g * (avg_gray / avg_g)).astype(np.uint8)
        r = np.minimum(255, r * (avg_gray / avg_r)).astype(np.uint8)
        return cv2.merge([b, g, r])

    def cbFindLane(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        if self.sub_image_type == 'raw':
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')
        
        # ✨ 1. 강화된 전처리 함수를 통과시킵니다. ✨
        cv_image_preprocessed = self.preprocess_image(cv_image)

        # ✨ 2. ROI는 전처리된 이미지를 대상으로 적용합니다. ✨
        roi_image = self.region_of_interest(cv_image_preprocessed)

        # (디버깅용 ROI 이미지 발행은 그대로 유지)
        self.pub_image_roi.publish(
            self.cvBridge.cv2_to_compressed_imgmsg(roi_image, 'jpg')
        )
        
        white_fraction, cv_white_lane = self.maskWhiteLane(roi_image)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(roi_image)


        try:
            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.fit_from_lines(
                    self.left_fit, cv_yellow_lane)
                self.mov_avg_left = np.append(
                    self.mov_avg_left, np.array([self.left_fit]), axis=0
                    )
            
            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.fit_from_lines(
                    self.right_fit, cv_white_lane)
                self.mov_avg_right = np.append(
                    self.mov_avg_right, np.array([self.right_fit]), axis=0
                    )

        except Exception:
            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
                self.mov_avg_left = np.array([self.left_fit])
            
            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
                self.mov_avg_right = np.array([self.right_fit])
        
        if yellow_fraction > 3000:
            self.left_fit_bef = self.left_fit
        if white_fraction > 3000:
            self.right_fit_bef = self.right_fit

        MOV_AVG_LENGTH = 5
        if self.mov_avg_left.shape[0] > 0:
            self.left_fit = np.array([
                np.mean(self.mov_avg_left[::-1][:, 0][0:min(MOV_AVG_LENGTH, self.mov_avg_left.shape[0])]),
                np.mean(self.mov_avg_left[::-1][:, 1][0:min(MOV_AVG_LENGTH, self.mov_avg_left.shape[0])]),
                np.mean(self.mov_avg_left[::-1][:, 2][0:min(MOV_AVG_LENGTH, self.mov_avg_left.shape[0])])
            ])
        if self.mov_avg_right.shape[0] > 0:
            self.right_fit = np.array([
                np.mean(self.mov_avg_right[::-1][:, 0][0:min(MOV_AVG_LENGTH, self.mov_avg_right.shape[0])]),
                np.mean(self.mov_avg_right[::-1][:, 1][0:min(MOV_AVG_LENGTH, self.mov_avg_right.shape[0])]),
                np.mean(self.mov_avg_right[::-1][:, 2][0:min(MOV_AVG_LENGTH, self.mov_avg_right.shape[0])])
            ])
        if self.mov_avg_left.shape[0] > 100:
            self.mov_avg_left = self.mov_avg_left[-MOV_AVG_LENGTH:]
        if self.mov_avg_right.shape[0] > 100:
            self.mov_avg_right = self.mov_avg_right[-MOV_AVG_LENGTH:]



        self.make_lane(cv_image, white_fraction, yellow_fraction) # 원본 이미지를 시각화에 사용

    # ✨ 새로운 전처리 함수: CLAHE 적용 ✨
    def preprocess_image(self, img):
        # LAB 색상 공간으로 변환 (L: 밝기, a,b: 색상)
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        
        # L 채널(밝기)에만 CLAHE를 적용하여 명암 대비를 평활화
        l_clahe = self.clahe.apply(l)
        
        # CLAHE가 적용된 L 채널과 원래 a, b 채널을 다시 병합
        lab_clahe = cv2.merge((l_clahe, a, b))
        
        # 다시 BGR 색상 공간으로 변환하여 반환
        return cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)



        
    def region_of_interest(self, image):
        height, width = image.shape[:2]
        top_y = int(height * 0.45)
        bottom_y = height
        top_left_x = int(width * 0.1)
        top_right_x = int(width * 0.9)
        bottom_left_x = 0
        bottom_right_x = width

        vertices = np.array([
            [(bottom_left_x, bottom_y)],
            [(bottom_right_x, bottom_y)],
            [(top_right_x, top_y)],
            [(top_left_x, top_y)]
        ], dtype=np.int32)
        
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, [vertices], (255, 255, 255))
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def maskWhiteLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([self.hue_white_l, self.saturation_white_l, self.lightness_white_l])
        upper_white = np.array([self.hue_white_h, self.saturation_white_h, self.lightness_white_h])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # ✨ 핵심 수정: 이미지 중앙을 기준으로 왼쪽 절반을 0으로 만들어 흰색 차선 후보에서 제외합니다.
        image_center = image.shape[1] // 2
        mask[:, :image_center] = 0

        kernel = np.ones((3,3),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        fraction_num = np.count_nonzero(mask)

        how_much_short = 0
        for i in range(0, image.shape[0]):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1
        how_much_short = image.shape[0] - how_much_short

        if how_much_short > 100:
            if self.reliability_white_line >= 5: self.reliability_white_line -= 5
        elif how_much_short <= 100:
            if self.reliability_white_line <= 99: self.reliability_white_line += 5

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line
        self.pub_white_line_reliability.publish(msg_white_line_reliability)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg')
                    )
            elif self.pub_image_type == 'raw':
                self.pub_image_white_lane.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, 'bgr8')
                    )
        return fraction_num, mask

    def maskYellowLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l])
        upper_yellow = np.array([self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # ✨ 핵심 수정: 이미지 중앙을 기준으로 오른쪽 절반을 0으로 만들어 노란색 차선 후보에서 제외합니다.
        image_center = image.shape[1] // 2
        mask[:, image_center:] = 0

        kernel = np.ones((3,3),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        fraction_num = np.count_nonzero(mask)

        how_much_short = 0
        for i in range(0, image.shape[0]):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1
        how_much_short = image.shape[0] - how_much_short

        if how_much_short > 100:
            if self.reliability_yellow_line >= 5: self.reliability_yellow_line -= 5
        elif how_much_short <= 100:
            if self.reliability_yellow_line <= 99: self.reliability_yellow_line += 5

        msg_yellow_line_reliability = UInt8()
        msg_yellow_line_reliability.data = self.reliability_yellow_line
        self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_yellow_lane.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg')
                    )
            elif self.pub_image_type == 'raw':
                self.pub_image_yellow_lane.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, 'bgr8')
                    )
        return fraction_num, mask

    def fit_from_lines(self, lane_fit, image):
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        lane_inds = (
            (nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) &
            (nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin))
            )
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]
        
        if len(x) == 0:
             return None, None # 피팅할 점이 없으면 None 반환
        
        lane_fit = np.polyfit(y, x, 2)
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
        return lane_fitx, lane_fit

    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        lane_base = 0
        
        if left_or_right == 'left':
            search_area_end = midpoint
            if histogram[0:search_area_end].size > 0:
                lane_base = np.argmax(histogram[0:search_area_end])
            else:
                self.get_logger().warn('Left lane search area is empty!')
                return self.left_fitx, self.left_fit_bef
        elif left_or_right == 'right':
            search_area_start = midpoint
            if histogram[search_area_start:].size > 0:
                lane_base = np.argmax(histogram[search_area_start:]) + search_area_start
            else:
                self.get_logger().warn('Right lane search area is empty!')
                return self.right_fitx, self.right_fit_bef

        nwindows = 20
        window_height = int(img_w.shape[0] / nwindows)
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        x_current = lane_base
        margin = 50
        minpix = 50
        lane_inds = []

        for window in range(nwindows):
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin
            good_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            lane_inds.append(good_lane_inds)
            if len(good_lane_inds) > minpix:
                x_current = int(np.mean(nonzerox[good_lane_inds]))

        lane_inds = np.concatenate(lane_inds)
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        lane_fit = self.left_fit_bef if left_or_right == 'left' else self.right_fit_bef
        if len(x) > 0:
            try:
                lane_fit = np.polyfit(y, x, 2)
            except Exception as e:
                self.get_logger().warn(f'Polyfit failed in sliding_window: {e}')

        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
        return lane_fitx, lane_fit
  
    def make_lane(self, cv_image, white_fraction, yellow_fraction):
        # 시각화를 위한 빈 이미지 생성
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))
        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        lane_state = UInt8()
        centerx = None

        # self.left_fitx와 self.right_fitx가 유효한 numpy 배열인지 확인
        is_yellow_detected = yellow_fraction > 3000 and self.left_fitx is not None
        is_white_detected = white_fraction > 3000 and self.right_fitx is not None

        # 1. 양쪽 차선 모두 감지 (가장 이상적인 상황)
        if is_yellow_detected and is_white_detected:
            lane_state.data = 2
            # 현재 차선 폭을 측정하여, 이동 평균으로 self.last_lane_offset을 부드럽게 업데이트
            current_offset = np.mean(self.right_fitx - self.left_fitx)
            self.last_lane_offset = (1 - self.lane_offset_alpha) * self.last_lane_offset + self.lane_offset_alpha * current_offset
            # 두 차선의 평균으로 중앙 차선 계산
            centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
            
            # 주행 영역 시각화
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # 2. 오른쪽 흰색 차선만 감지 (커브 길 등)
        elif is_white_detected:
            lane_state.data = 3
            # 안정적으로 유지되던 차선 폭 정보를 이용해 왼쪽 차선 위치와 중앙선을 추정
            centerx = self.right_fitx - (self.last_lane_offset / 2)
            self.get_logger().info(f'White lane only. Recovering center using offset: {self.last_lane_offset:.2f}')

        # 3. 왼쪽 노란색 차선만 감지 (커브 길 등)
        elif is_yellow_detected:
            lane_state.data = 1
            # 안정적으로 유지되던 차선 폭 정보를 이용해 오른쪽 차선 위치와 중앙선을 추정
            centerx = self.left_fitx + (self.last_lane_offset / 2)
            self.get_logger().info(f'Yellow lane only. Recovering center using offset: {self.last_lane_offset:.2f}')
            
        # 4. 양쪽 차선 모두 감지 실패 (최악의 상황)
        else:
            lane_state.data = 0
            # 차선을 놓쳤을 때 급격히 방향을 트는 것을 방지하기 위해, 마지막으로 유효했던 중앙선을 그대로 유지
            centerx = self.last_known_centerx
            self.get_logger().warn('No lanes detected! Maintaining last known center.')

        # 중앙 차선 시각화 및 조향 값 발행
        if centerx is not None:
            # 마지막으로 유효했던 중앙선 정보를 계속 업데이트 (끊김 없는 주행 경로 생성)
            self.last_known_centerx = centerx
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)
            
            msg_desired_center = Float64()
            # 이미지 하단부(로봇과 가까운)의 중앙값을 조향 값으로 사용
            msg_desired_center.data = centerx[int(cv_image.shape[0] * 0.9)]
            self.pub_lane.publish(msg_desired_center)

        self.pub_lane_state.publish(lane_state)

        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        if self.pub_image_type == 'compressed':
            self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))
        elif self.pub_image_type == 'raw':
            self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()