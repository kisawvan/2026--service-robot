#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import sys
from collections import deque

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class RadarSelfCheck(object):
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan_filtered")
        self.raw_scan_topic = rospy.get_param("~raw_scan_topic", "/scan")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.sensor_timeout = float(rospy.get_param("~sensor_timeout", 8.0))
        self.observe_window = float(rospy.get_param("~observe_window", 10.0))
        self.front_angle_deg = float(rospy.get_param("~front_angle_deg", 20.0))
        self.min_obstacle_distance = float(rospy.get_param("~min_obstacle_distance", 0.60))
        self.stop_vel_threshold = float(rospy.get_param("~stop_vel_threshold", 0.05))

        self.scan_times = deque(maxlen=200)
        self.front_samples = deque(maxlen=500)
        self.cmd_samples = deque(maxlen=500)
        self.latest_scan = None

        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=20)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_cb, queue_size=50)

        self.results = []

    def _scan_cb(self, msg):
        now = rospy.Time.now().to_sec()
        self.latest_scan = msg
        self.scan_times.append(now)
        front_min = self._front_min_distance(msg, math.radians(self.front_angle_deg))
        self.front_samples.append((now, front_min))

    def _cmd_cb(self, msg):
        now = rospy.Time.now().to_sec()
        self.cmd_samples.append((now, msg.linear.x, msg.angular.z))

    def _front_min_distance(self, scan_msg, front_half_angle_rad):
        min_dist = float("inf")
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if abs(angle) <= front_half_angle_rad:
                finite = (not math.isinf(r)) and (not math.isnan(r))
                if finite and scan_msg.range_min <= r <= scan_msg.range_max:
                    if r < min_dist:
                        min_dist = r
            angle += scan_msg.angle_increment
        return min_dist

    def _record(self, level, name, detail):
        self.results.append((level, name, detail))
        if level == "PASS":
            rospy.loginfo("[PASS] %s: %s", name, detail)
        elif level == "WARN":
            rospy.logwarn("[WARN] %s: %s", name, detail)
        else:
            rospy.logerr("[FAIL] %s: %s", name, detail)

    def _check_topic_exists(self, topic_name):
        topics = rospy.get_published_topics()
        topic_list = [name for name, _ in topics]
        if topic_name in topic_list:
            self._record("PASS", "话题存在", "%s 已发布" % topic_name)
            return True
        self._record("FAIL", "话题存在", "%s 未发布" % topic_name)
        return False

    def _wait_for_scan(self):
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if len(self.scan_times) >= 3:
                break
            if rospy.Time.now().to_sec() - start > self.sensor_timeout:
                break
            rate.sleep()

        if len(self.scan_times) < 3 or self.latest_scan is None:
            self._record("FAIL", "雷达数据流", "在 %.1f 秒内未收到足够的 %s 数据" % (self.sensor_timeout, self.scan_topic))
            return False

        dt = self.scan_times[-1] - self.scan_times[0]
        hz = 0.0
        if dt > 0.0:
            hz = float(len(self.scan_times) - 1) / dt
        self._record("PASS", "雷达数据流", "收到 %d 帧，估计频率 %.2f Hz" % (len(self.scan_times), hz))
        return True

    def _check_param_equal(self, param_name, expected, critical=True):
        if not rospy.has_param(param_name):
            level = "FAIL" if critical else "WARN"
            self._record(level, "参数检查", "%s 不存在" % param_name)
            return False
        value = rospy.get_param(param_name)
        if value == expected:
            self._record("PASS", "参数检查", "%s=%s" % (param_name, str(value)))
            return True
        level = "FAIL" if critical else "WARN"
        self._record(level, "参数检查", "%s=%s，期望=%s" % (param_name, str(value), str(expected)))
        return False

    def _check_param_predicate(self, param_name, predicate, expect_desc, critical=True):
        if not rospy.has_param(param_name):
            level = "FAIL" if critical else "WARN"
            self._record(level, "参数检查", "%s 不存在" % param_name)
            return False
        value = rospy.get_param(param_name)
        if predicate(value):
            self._record("PASS", "参数检查", "%s=%s（%s）" % (param_name, str(value), expect_desc))
            return True
        level = "FAIL" if critical else "WARN"
        self._record(level, "参数检查", "%s=%s，不满足：%s" % (param_name, str(value), expect_desc))
        return False

    def _check_runtime_behavior(self):
        rospy.loginfo("开始运行时观测：%.1f 秒。请在机器人前方放置障碍物以验证避障。", self.observe_window)
        start_t = rospy.Time.now().to_sec()
        end_t = start_t + self.observe_window
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < end_t:
            rate.sleep()

        obstacle_times = []
        for t, d in list(self.front_samples):
            if start_t <= t <= end_t and d < self.min_obstacle_distance:
                obstacle_times.append(t)

        if not obstacle_times:
            self._record(
                "WARN",
                "运行时避障",
                "观测窗口内未检测到前方 %.2f m 内障碍物，无法完成动态避障判定" % self.min_obstacle_distance,
            )
            return True

        if not self.cmd_samples:
            self._record("FAIL", "运行时避障", "检测到障碍物，但未收到任何 %s 数据" % self.cmd_vel_topic)
            return False

        violation_count = 0
        for obs_t in obstacle_times:
            nearby_cmd = [c for c in self.cmd_samples if abs(c[0] - obs_t) <= 0.5]
            if not nearby_cmd:
                continue
            max_forward = max([c[1] for c in nearby_cmd])
            if max_forward > self.stop_vel_threshold:
                violation_count += 1

        if violation_count > 0:
            self._record(
                "FAIL",
                "运行时避障",
                "障碍出现时仍存在前进速度 > %.3f m/s 的控制指令（共 %d 次）" % (self.stop_vel_threshold, violation_count),
            )
            return False

        self._record("PASS", "运行时避障", "障碍出现期间未发现超阈值前进指令")
        return True

    def run(self):
        rospy.loginfo("=== 雷达避障自检开始 ===")
        hard_fail = False

        if not self._check_topic_exists(self.raw_scan_topic):
            hard_fail = True
        if not self._check_topic_exists(self.scan_topic):
            hard_fail = True
        if not self._check_topic_exists(self.cmd_vel_topic):
            hard_fail = True

        if not self._wait_for_scan():
            hard_fail = True

        if not self._check_param_equal("/move_base/local_costmap/obstacle_layer/enabled", True, critical=True):
            hard_fail = True
        if not self._check_param_predicate(
            "/move_base/local_costmap/obstacle_layer/scan/topic",
            lambda v: str(v).lstrip("/") == self.scan_topic.lstrip("/"),
            "应与 scan_topic 对齐",
            critical=True,
        ):
            hard_fail = True
        if not self._check_param_equal("/move_base/local_costmap/obstacle_layer/scan/marking", True, critical=True):
            hard_fail = True
        if not self._check_param_equal("/move_base/local_costmap/obstacle_layer/scan/clearing", True, critical=True):
            hard_fail = True
        if not self._check_param_predicate(
            "/move_base/local_costmap/obstacle_layer/obstacle_range",
            lambda v: float(v) > 0.0,
            "应大于 0",
            critical=True,
        ):
            hard_fail = True
        if not self._check_param_predicate(
            "/move_base/base_local_planner",
            lambda v: "DWAPlannerROS" in str(v) or "teb_local_planner" in str(v),
            "应启用局部规划器",
            critical=False,
        ):
            pass

        if not self._check_runtime_behavior():
            hard_fail = True

        pass_count = len([r for r in self.results if r[0] == "PASS"])
        warn_count = len([r for r in self.results if r[0] == "WARN"])
        fail_count = len([r for r in self.results if r[0] == "FAIL"])

        rospy.loginfo("=== 雷达避障自检结束：PASS=%d WARN=%d FAIL=%d ===", pass_count, warn_count, fail_count)
        if hard_fail:
            rospy.logerr("结论：雷达避障当前不可用或配置不完整。")
            return 2
        rospy.loginfo("结论：雷达避障具备实现条件。")
        return 0


def main():
    rospy.init_node("radar_self_check")
    checker = RadarSelfCheck()
    code = checker.run()
    sys.exit(code)


if __name__ == "__main__":
    main()
