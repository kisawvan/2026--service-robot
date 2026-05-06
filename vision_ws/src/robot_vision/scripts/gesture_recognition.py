#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import math
from collections import Counter, deque

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import mediapipe as mp
except Exception:
    mp = None


def imgmsg_to_cv2(img_msg):
    # 抛弃 cv_bridge，直接解析二进制数据，避免 ROS Melodic 下的 Python3 兼容问题
    return np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                      dtype=np.uint8, buffer=img_msg.data)


def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height, img_msg.width, _ = cv_image.shape
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg


class GestureRecognitionCV:
    def __init__(self):
        rospy.init_node('gesture_recognition_cv_node', anonymous=True)

        # ROS params
        self.debug = rospy.get_param('~debug', False)
        self.backend = rospy.get_param('~backend', 'auto')  # auto / mediapipe / opencv
        self.use_backproj = rospy.get_param('~use_backproj', True)
        self.min_area = rospy.get_param('~min_area', 2000)
        self.history_len = rospy.get_param('~history_len', 7)
        self.min_detection_confidence = rospy.get_param('~min_detection_confidence', 0.65)
        self.min_tracking_confidence = rospy.get_param('~min_tracking_confidence', 0.65)
        self.result_topic = rospy.get_param('~result_topic', '/gesture_recognition/result')
        self.image_topic = rospy.get_param('~image_topic', '/gesture_recognition/image')

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10)
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=10)

        self.result_history = deque(maxlen=self.history_len)
        self.last_published = None

        # 旧方案兜底使用的肤色直方图
        self.skin_hist = None

        self.use_mediapipe = False
        self.mp_hands = None
        self.mp_drawing = None
        self.mp_styles = None
        self.hands = None

        if self.backend in ('auto', 'mediapipe') and mp is not None:
            self.mp_hands = mp.solutions.hands
            self.mp_drawing = mp.solutions.drawing_utils
            self.mp_styles = mp.solutions.drawing_styles
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                model_complexity=1,
                max_num_hands=1,
                min_detection_confidence=self.min_detection_confidence,
                min_tracking_confidence=self.min_tracking_confidence,
            )
            self.use_mediapipe = True

        if self.debug:
            cv2.namedWindow('gesture_debug', cv2.WINDOW_NORMAL)

        if self.use_mediapipe:
            rospy.loginfo('Gesture node started with MediaPipe Hands backend.')
        else:
            rospy.logwarn('MediaPipe unavailable, fallback to OpenCV contour backend.')

    def __del__(self):
        try:
            if self.hands is not None:
                self.hands.close()
        except Exception:
            pass

    def _normalize_label(self, label):
        mapping = {
            'FIST': 'FIST',
            'PALM': 'PALM',
            'V_SIGN': 'V_SIGN',
            'POINT': 'POINT',
            'THUMB_UP': 'THUMB_UP',
            'THUMB_DOWN': 'THUMB_DOWN',
            'OK': 'OK',
            'THREE': 'THREE',
            'FOUR': 'FOUR',
            'UNKNOWN': 'UNKNOWN',
            'NONE': 'NONE',
        }
        return mapping.get(label, label)

    def _classify_landmarks(self, landmarks, handedness='Unknown'):
        pts = landmarks.landmark

        def dist(a, b):
            return math.hypot(a.x - b.x, a.y - b.y)

        wrist = pts[0]
        index_mcp, index_pip, index_tip = pts[5], pts[6], pts[8]
        middle_mcp, middle_pip, middle_tip = pts[9], pts[10], pts[12]
        ring_mcp, ring_pip, ring_tip = pts[13], pts[14], pts[16]
        pinky_mcp, pinky_pip, pinky_tip = pts[17], pts[18], pts[20]
        thumb_mcp, thumb_ip, thumb_tip = pts[2], pts[3], pts[4]

        # 以手掌尺度归一化，避免远近变化导致阈值漂移
        palm_scale = max(dist(wrist, middle_mcp), 1e-6)

        index_up = index_tip.y < index_pip.y and index_tip.y < index_mcp.y
        middle_up = middle_tip.y < middle_pip.y and middle_tip.y < middle_mcp.y
        ring_up = ring_tip.y < ring_pip.y and ring_tip.y < ring_mcp.y
        pinky_up = pinky_tip.y < pinky_pip.y and pinky_tip.y < pinky_mcp.y

        if handedness == 'Right':
            thumb_extended = thumb_tip.x < thumb_ip.x < thumb_mcp.x
        elif handedness == 'Left':
            thumb_extended = thumb_tip.x > thumb_ip.x > thumb_mcp.x
        else:
            thumb_extended = dist(thumb_tip, thumb_ip) > dist(thumb_ip, thumb_mcp) * 0.8

        finger_states = [thumb_extended, index_up, middle_up, ring_up, pinky_up]
        finger_count = sum(1 for state in finger_states if state)

        thumb_index_dist = dist(thumb_tip, index_tip) / palm_scale
        all_curled = not any([index_up, middle_up, ring_up, pinky_up])
        three_up = index_up and middle_up and ring_up and not pinky_up

        if thumb_index_dist < 0.33 and not ring_up and not pinky_up and not middle_up:
            return 'OK', 0.95
        if finger_count == 0:
            return 'FIST', 0.96
        if finger_count >= 4:
            return 'PALM', 0.94
        if index_up and middle_up and not ring_up and not pinky_up and not thumb_extended:
            return 'V_SIGN', 0.93
        if index_up and not middle_up and not ring_up and not pinky_up:
            return 'POINT', 0.92
        if thumb_extended and not index_up and not middle_up and not ring_up and not pinky_up:
            return 'THUMB_UP' if thumb_tip.y < wrist.y else 'THUMB_DOWN', 0.90
        if three_up:
            return 'THREE', 0.88
        if finger_count == 4:
            return 'FOUR', 0.88
        if all_curled:
            return 'FIST', 0.90
        return 'UNKNOWN', 0.50

    def _classify_contour(self, frame):
        # 保底方案：仍保留旧的肤色/轮廓逻辑，避免依赖不可用时完全失效
        if self.skin_hist is not None and self.use_backproj:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            backproj = cv2.calcBackProject([hsv], [0, 1], self.skin_hist, [0, 180, 0, 256], 1)
            disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
            cv2.filter2D(backproj, -1, disc, backproj)
            _, mask = cv2.threshold(backproj, 50, 255, cv2.THRESH_BINARY)
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)

            lower_hsv = np.array([0, 20, 40])
            upper_hsv = np.array([25, 200, 255])
            mask_hsv = cv2.inRange(hsv, lower_hsv, upper_hsv)

            lower_ycrcb = np.array([0, 133, 77])
            upper_ycrcb = np.array([255, 173, 127])
            mask_ycrcb = cv2.inRange(ycrcb, lower_ycrcb, upper_ycrcb)

            mask = cv2.bitwise_and(mask_hsv, mask_ycrcb)

        mask = cv2.GaussianBlur(mask, (7, 7), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        gesture_result = 'NONE'
        debug_overlay = frame.copy()

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) > self.min_area:
                cv2.drawContours(debug_overlay, [cnt], -1, (0, 255, 0), 2)
                hull_points = cv2.convexHull(cnt)
                cv2.drawContours(debug_overlay, [hull_points], -1, (255, 0, 0), 2)
                gesture_result = 'PALM' if len(hull_points) > 5 else 'UNKNOWN'

        return gesture_result, 0.55, debug_overlay, mask

    def _smooth_result(self, raw_result):
        if raw_result == 'UNKNOWN':
            return self.last_published or 'UNKNOWN'

        self.result_history.append(raw_result)
        most_common = Counter(self.result_history).most_common(1)
        return most_common[0][0] if most_common else raw_result

    def _publish_result(self, result):
        if result != self.last_published:
            self.result_pub.publish(result)
            self.last_published = result

    def image_callback(self, data):
        try:
            frame = imgmsg_to_cv2(data).copy()
        except Exception as e:
            rospy.logerr('Image Convert Error: %s', str(e))
            return

        display = frame.copy()
        raw_result = 'NONE'
        confidence = 0.0

        if self.use_mediapipe:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb_frame.flags.writeable = False
            results = self.hands.process(rgb_frame)
            rgb_frame.flags.writeable = True

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                handedness = 'Unknown'
                if results.multi_handedness:
                    handedness = results.multi_handedness[0].classification[0].label

                raw_result, confidence = self._classify_landmarks(hand_landmarks, handedness)
                display = frame.copy()
                self.mp_drawing.draw_landmarks(
                    display,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_styles.get_default_hand_landmarks_style(),
                    self.mp_styles.get_default_hand_connections_style(),
                )
            else:
                raw_result = 'NONE'
                confidence = 0.0
        else:
            raw_result, confidence, display, mask = self._classify_contour(frame)
            if self.debug:
                cv2.imshow('mask_debug', mask)

        smooth_result = self._smooth_result(raw_result)
        self._publish_result(smooth_result)

        cv2.putText(display, 'Gesture: {}'.format(smooth_result), (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        cv2.putText(display, 'Backend: {}  Conf: {:.2f}'.format('MediaPipe' if self.use_mediapipe else 'OpenCV', confidence),
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 180, 0), 2)

        if self.debug:
            cv2.imshow('gesture_debug', display)
            cv2.waitKey(1)

        try:
            self.image_pub.publish(cv2_to_imgmsg(display))
        except Exception as e:
            rospy.logerr(str(e))


if __name__ == '__main__':
    try:
        GestureRecognitionCV()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass