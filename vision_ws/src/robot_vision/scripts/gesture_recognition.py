#!/usr/bin/env python3
# 简单高精度 OpenCV 手势识别（非 CNN 实现）
import cv2
import numpy as np
import math
import time

try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    ROS_ENABLED = True
except Exception:
    ROS_ENABLED = False


class GestureRecognitionOpenCV:
    """基于肤色直方图校准 + 轮廓 + 凸缺陷计数的手势识别器。

    使用方法：运行脚本后摄像头画面会显示一个绿色校准框，按 `c` 在框内采样肤色，
    然后将实时分割手部并统计手指数量；按 `q` 退出。
    """

    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        self.hist = None
        self.roi_box = (300, 100, 100, 100)  # x,y,w,h for calibration box
        self.min_contour_area = 2000
        if ROS_ENABLED:
            rospy.init_node('gesture_recognition_opencv_node', anonymous=True)
            self.result_pub = rospy.Publisher('/gesture_recognition/result', String, queue_size=1)

    def calibrate_from_frame(self, frame):
        x, y, w, h = self.roi_box
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0:
            return
        ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
        # 计算 Cr、Cb 的二维直方图
        hist = cv2.calcHist([ycrcb], [1, 2], None, [128, 128], [0, 256, 0, 256])
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
        self.hist = hist

    def segment_hand(self, frame):
        # 先使用直方图回投（若已校准），否则使用 HSV 简单阈值作为后备
        blur = cv2.GaussianBlur(frame, (7, 7), 0)
        if self.hist is not None:
            ycrcb = cv2.cvtColor(blur, cv2.COLOR_BGR2YCrCb)
            backproj = cv2.calcBackProject([ycrcb], [1, 2], self.hist, [0, 256, 0, 256], 1)
            # 平滑并二值化
            disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
            backproj = cv2.filter2D(backproj, -1, disc)
            _, mask = cv2.threshold(backproj, 50, 255, cv2.THRESH_BINARY)
        else:
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            lower = np.array([0, 30, 60])
            upper = np.array([25, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.GaussianBlur(mask, (7, 7), 0)
        _, mask = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
        return mask

    @staticmethod
    def count_fingers_from_contour(contour, drawing=None):
        # 参考凸缺陷法，返回手指计数（保守估计）
        hull = cv2.convexHull(contour, returnPoints=False)
        if hull is None or len(hull) < 3:
            return 0
        defects = cv2.convexityDefects(contour, hull)
        if defects is None:
            return 0

        cnt = 0
        bounding_rect = cv2.boundingRect(contour)
        _, _, w, h = bounding_rect
        depth_threshold = max(20, h * 0.06)

        for i in range(defects.shape[0]):
            s, e, f, depth = defects[i, 0]
            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            far = tuple(contour[f][0])
            a = math.dist(start, end)
            b = math.dist(start, far)
            c = math.dist(end, far)
            # 用余弦定理计算夹角
            if b * c == 0:
                continue
            angle = math.degrees(math.acos(max(-1.0, min(1.0, (b*b + c*c - a*a) / (2*b*c)))))
            if angle < 90 and depth > depth_threshold:
                cnt += 1
                if drawing is not None:
                    cv2.circle(drawing, far, 5, (0, 255, 0), -1)

        # defects count + 1 ≈ fingers
        return min(5, cnt + 1)

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            vis = frame.copy()
            x, y, w, h = self.roi_box
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)

            mask = self.segment_hand(frame)
            
            # 兼容不同 OpenCV 版本的 findContours
            contour_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contour_result) == 3:
                # OpenCV 3.x: 返回 (image, contours, hierarchy)
                contours = contour_result[1]
            else:
                # OpenCV 4.x+: 返回 (contours, hierarchy)
                contours = contour_result[0]
            
            gesture_text = 'None'

            if contours:
                # 选取最大轮廓
                contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(contour)
                if area > self.min_contour_area:
                    hull = cv2.convexHull(contour)
                    cv2.drawContours(vis, [contour], -1, (255, 0, 0), 2)
                    cv2.drawContours(vis, [hull], -1, (0, 0, 255), 2)
                    finger_count = self.count_fingers_from_contour(contour, drawing=vis)
                    if finger_count == 0:
                        gesture_text = 'Fist'
                    elif finger_count == 1:
                        gesture_text = 'One'
                    elif finger_count == 2:
                        gesture_text = 'Two'
                    elif finger_count == 3:
                        gesture_text = 'Three'
                    elif finger_count == 4:
                        gesture_text = 'Four'
                    else:
                        gesture_text = 'Five'

                    if ROS_ENABLED:
                        try:
                            self.result_pub.publish(gesture_text)
                        except Exception:
                            pass

            cv2.putText(vis, 'Gesture: ' + gesture_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(vis, "Press 'c' to calibrate, 'q' to quit", (10, vis.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

            cv2.imshow('Gesture (OpenCV)', vis)
            cv2.imshow('Mask', mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                # 采样多帧并平均直方图以提高鲁棒性
                samples = []
                for i in range(8):
                    ret, f = self.cap.read()
                    if not ret:
                        break
                    f = cv2.flip(f, 1)
                    x, y, w, h = self.roi_box
                    roi = f[y:y+h, x:x+w]
                    samples.append(roi)
                if samples:
                    combined = np.vstack([cv2.cvtColor(s, cv2.COLOR_BGR2YCrCb) for s in samples])
                    hist = cv2.calcHist([combined], [1, 2], None, [128, 128], [0, 256, 0, 256])
                    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
                    self.hist = hist
            elif key == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    gr = GestureRecognitionOpenCV(camera_id=0)
    gr.run()