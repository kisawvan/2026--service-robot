#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import math
import tf.transformations as tf_trans
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from robot_vision.msg import AprilTagDetection
from xml.dom.minidom import parse
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus


# 8.12 sanzhe meijiazhongjiandian
class NavigationController:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))

        # 导航状态控制
        self.is_navigating = False
        self.current_position = [0.0, 0.0]  # 当前位置

        # AprilTag检测相关
        self.tag_id = None
        self.waiting_for_tag = False
        self.tag_sub = rospy.Subscriber("/apriltag_detector/detections",
                                        AprilTagDetection,
                                        self.tag_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 订阅里程计获取位置
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 新增：订阅按键信息
        self.key_sub = rospy.Subscriber("key_input", String, self.key_callback)

    def odom_callback(self, msg):
        """更新当前位置"""
        try:
            self.current_position[0] = msg.pose.pose.position.x
            self.current_position[1] = msg.pose.pose.position.y
        except Exception as e:
            rospy.logwarn("无法更新位置: %s", str(e))

    def tag_callback(self, msg):
        """处理二维码检测结果"""
        if self.waiting_for_tag:
            self.tag_id = msg.tag_id
            rospy.loginfo("检测到AprilTag ID: %d", self.tag_id)
            self.waiting_for_tag = False

    def key_callback(self, msg):
        """处理按键信息回调"""
        self.key = msg.data  # 这里进行赋值

    def parse_xml(self, xml_file):
        """解析XML导航文件"""
        try:
            doc = parse(xml_file)
            root = doc.documentElement
            points = root.getElementsByTagName("Waypoint")
            return [
                [
                    float(p.getElementsByTagName("Pos_x")[0].childNodes[0].data),
                    float(p.getElementsByTagName("Pos_y")[0].childNodes[0].data),
                    float(p.getElementsByTagName("Pos_z")[0].childNodes[0].data),
                    float(p.getElementsByTagName("Ori_x")[0].childNodes[0].data),
                    float(p.getElementsByTagName("Ori_y")[0].childNodes[0].data),
                    float(p.getElementsByTagName("Ori_z")[0].childNodes[0].data),
                    float(p.getElementsByTagName("Ori_w")[0].childNodes[0].data)
                ] for p in points
            ]
        except Exception as e:
            rospy.logerr("XML解析错误: %s", str(e))
            return []

    def generate_special_path(self, start_x, start_y, idex, forward_length=0.8):
        """生成特殊弓字形路径点序列 (参考C++实现)"""
        # 固定侧向移动距离
        first_side_length1 = 0.27  # 15cm
        second_side_length1 = 0.26  # 20cm
        first_side_length2 = 0.27  # 15cm
        second_side_length2 = 0.27  # 20cm
        first_side_length3 = 0.27  # 15cm
        second_side_length3 = 0.26

        first_side_length4 = 0.27  # 15cm
        second_side_length4 = 0.26  # 20cm
        first_side_length5 = 0.22  # 15cm
        second_side_length5 = 0.22  # 20cm
        first_side_length6 = 0.20  # 15cm
        second_side_length6 = 0.26

        third_side_length = 0.20  # 20cm
        fourth_side_length = 0.15 # 15cm
        addition_side_length = 0.10  # 15cm
        front_quat = tf_trans.quaternion_from_euler(0, 0, 0)  # 正0度
        back_quat = tf_trans.quaternion_from_euler(0, 0, math.pi)  # 反180度
        right_quat = tf_trans.quaternion_from_euler(0, 0, -math.pi / 2)  # 右90度
        left_quat = tf_trans.quaternion_from_euler(0, 0, math.pi / 2)  # 左90度

        if idex == 1 :
            return [
                [start_x, start_y, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x + forward_length, start_y, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                [start_x + forward_length, start_y + first_side_length1, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x, start_y + first_side_length1, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                [start_x, start_y + first_side_length1 + second_side_length1, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x + forward_length, start_y + first_side_length1 + second_side_length1, 0.0, left_quat[0],
                 left_quat[1], left_quat[2], left_quat[3]]
            ]
        elif idex == 2 :
            return [
                [start_x, start_y, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x - forward_length, start_y, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                [start_x - forward_length, start_y + first_side_length2, 0.0, front_quat[0], front_quat[1], front_quat[2],
                 front_quat[3]],

                [start_x, start_y + first_side_length2, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                [start_x, start_y + first_side_length2 + second_side_length2, 0.0, front_quat[0], front_quat[1],
                 front_quat[2], front_quat[3]],

                [start_x - forward_length, start_y + first_side_length2 + second_side_length2, 0.0, front_quat[0],
                 front_quat[1], front_quat[2], front_quat[3]]
            ]
        elif idex == 3 :
            return [
                [start_x, start_y, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                [start_x - forward_length, start_y , 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]],

                [start_x - forward_length, start_y - first_side_length3 , 0.0, back_quat[0], back_quat[1], back_quat[2],
                 back_quat[3]],

                [start_x, start_y - first_side_length3, 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]],

                [start_x, start_y - second_side_length3 - first_side_length3 , 0.0, back_quat[0],
                 back_quat[1], back_quat[2], back_quat[3]],

                [start_x - forward_length, start_y - second_side_length3 - first_side_length3 , 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]]
            ]
        elif idex == 4:
            return [
                [start_x, start_y, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                [start_x + forward_length, start_y, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                [start_x + forward_length, start_y + first_side_length4, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                [start_x, start_y + first_side_length4, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                [start_x, start_y + first_side_length4 + second_side_length4, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                [start_x + forward_length, start_y + first_side_length4 + second_side_length4, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]]
            ]
        elif idex == 5:
            return [
                [start_x, start_y, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x + forward_length, start_y, 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]],

                [start_x + forward_length, start_y - first_side_length5, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x, start_y - first_side_length5, 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]],

                [start_x, start_y - first_side_length5 - second_side_length5, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x + forward_length, start_y - first_side_length5 - second_side_length5, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]]
            ]
        elif idex == 6:
            return [

                [start_x, start_y, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x - forward_length, start_y, 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]],

                [start_x - forward_length, start_y - first_side_length6, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x, start_y - first_side_length6, 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]],

                [start_x, start_y - first_side_length6 - second_side_length6, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                [start_x - forward_length, start_y - first_side_length6 - second_side_length6, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]]
            ]

    def navigate_to_point(self, point, index, total):
        """导航到精确坐标点"""
        rospy.loginfo("导航至航点 %d/%d", index + 1, total)

        # 创建目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.position.z = point[2]
        goal.target_pose.pose.orientation.x = point[3]
        goal.target_pose.pose.orientation.y = point[4]
        goal.target_pose.pose.orientation.z = point[5]
        goal.target_pose.pose.orientation.w = point[6]

        # 发送目标
        self.move_base.send_goal(goal)
        success = self.move_base.wait_for_result(rospy.Duration(60))

        # 状态检查
        if not success:
            rospy.logwarn("到达目标点超时，尝试继续执行")
            return False
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("成功到达目标点!")
                return True
            else:
                rospy.logwarn("导航失败，错误码: %d", state)
                return False

    def execute_special_path(self, start_x, start_y, idex):
        """执行精确的弓字形路径"""
        rospy.loginfo("执行弓字形路径 (起始点: %.2f, %.2f)", start_x, start_y)

        # 获取路径点
        if idex == 1 :
            path_points = self.generate_special_path(start_x, start_y, 1)
        elif idex == 2 :
            path_points = self.generate_special_path(start_x, start_y, 2)
        elif idex == 3:
            path_points = self.generate_special_path(start_x, start_y, 3)
        elif idex == 4 :
            path_points = self.generate_special_path(start_x, start_y, 4)
        elif idex == 5 :
            path_points = self.generate_special_path(start_x, start_y, 5)
        elif idex == 6 :
            path_points = self.generate_special_path(start_x, start_y, 6)

        # 执行所有路径点
        for i, point in enumerate(path_points):
            rospy.sleep(0.04)
            if not self.navigate_to_point(point, i + 1, len(path_points) + 1):
                rospy.logwarn("路径点 %d 导航失败", i + 1)
                return False

        return True

    def navigate_points(self, points, id):
        """导航主航点并在关键点执行区域覆盖"""
        front_quat = tf_trans.quaternion_from_euler(0, 0, 0)  # 正0度
        back_quat = tf_trans.quaternion_from_euler(0, 0, math.pi)  # 反180度
        right_quat = tf_trans.quaternion_from_euler(0, 0, -math.pi / 2)  # 右90度
        left_quat = tf_trans.quaternion_from_euler(0, 0, math.pi / 2)  # 左90度

        if id == 1:
            self.navigate_to_point([1.32, -0.12, 0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]], 1, 3)  # 避免撞箱子
            self.execute_special_path(points[0][0], points[0][1], 1)

            self.navigate_to_point([2.4, 1.4, 0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]], 1, 3)
	    self.navigate_to_point([2.45, 1.7, 0, left_quat[0], left_quat[1], left_quat[2],left_quat[3]], 1, 3)  # 避免撞箱子
            self.execute_special_path(points[1][0], points[1][1], 2)

            self.navigate_to_point([1.4, 2.45, 0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]], 1, 3)  # 避免撞箱子
            self.execute_special_path(points[2][0], points[2][1], 3)
    	    self.navigate_to_point([0.02, 2.40, 0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]], 1, 3)
            self.navigate_to_point([0.02, 1.40, 0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]], 1, 3)

        elif id == 3:
            self.navigate_to_point([0.1, 0.25, 0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]], 1, 3)
            self.navigate_to_point([0.1, 1.5, 0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]], 1, 3)
            self.execute_special_path(points[0][0], points[0][1], 4)

            self.navigate_to_point([1.19, 2.46, 0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]], 1, 3)  # 避免撞箱子
	    self.navigate_to_point([1.5, 2.46, 0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]], 1, 3)
            self.execute_special_path(points[1][0], points[1][1], 5)

            self.navigate_to_point([2.45, 1.51, 0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]], 1, 3)  # 定点1
	    self.navigate_to_point([2.45, 0.1, 0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]], 1, 3)
            self.execute_special_path(points[2][0], points[2][1], 6)

            self.navigate_to_point([1.91, 0.029, 0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]], 1, 3)  #定点2

    def start_navigation(self):
        if self.is_navigating:
            rospy.logwarn("导航正在进行中，忽略新请求")
            return

        self.is_navigating = True
        rospy.loginfo("开始导航流程")
        # 如果按下 'g'，优先前往 (1.9, 1.87)，并跳过起点二维码检测
        xml_file = ""
        if getattr(self, 'key', None) == "g":
            xml_file = "/home/bcsh/waypoints3_1.xml"
            try:
                init_quat = tf_trans.quaternion_from_euler(0, 0, 0)
                init_point = [1.9, 1.87, 0, init_quat[0], init_quat[1], init_quat[2], init_quat[3]]
                rospy.loginfo("按下'g'，跳过检测，先前往 (1.9, 1.87)")
                self.navigate_to_point(init_point, 0, 1)
                rospy.sleep(0.05)
            except Exception as e:
                rospy.logwarn("前往初始化点失败: %s", str(e))
        else:
            # 1. 二维码检测准备
            rospy.loginfo("准备二维码检测...")
            self.waiting_for_tag = True
            self.tag_id = None

            # 2. 移动到检测位置
            quat = tf_trans.quaternion_from_euler(0, 0, -math.pi / 2)  # 90
            detection_point = [0.65, 0.0, 0, quat[0], quat[1], quat[2], quat[3]]
            self.navigate_to_point(detection_point, 0, 1)

            # 3. 等待二维码检测 (最多5秒)
            timeout = 5.0
            start_time = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - start_time < timeout and
                   self.waiting_for_tag and not rospy.is_shutdown()):
                rospy.sleep(0.1)

            # 4. 根据检测结果选择路径
            if self.key == "g":
                if self.tag_id == 1:
                    xml_file = "/home/bcsh/waypoints3_1.xml"
                elif self.tag_id == 3:
                    xml_file = "/home/bcsh/waypoints3_3.xml"
                else:
                    xml_file = "/home/bcsh/waypoints3_1.xml"
            elif self.key == "h":
                xml_file = "/home/bcsh/waypoints3_1.xml"
                self.tag_id = 1
            elif self.key == "j":
                xml_file = "/home/bcsh/waypoints3_3.xml"
                self.tag_id = 3

        # 5. 解析并执行路径
        points = self.parse_xml(xml_file)
        if points:
            # 增强健壮性：确保索引存在
            try:
                self.navigate_points(points, getattr(self, 'tag_id', 1))
            except Exception as e:
                rospy.logwarn("执行导航时出错: %s", str(e))
        else:
            rospy.logwarn("导航点列表为空")

        # 返回起始位置（如果存在）
        if points and len(points) > 3:
            back_quat = tf_trans.quaternion_from_euler(0, 0, math.pi)
            self.navigate_to_point(points[3], 0, 1)

        self.is_navigating = False
        rospy.loginfo("导航流程完成!")


def callback(msg, controller):
    rospy.loginfo("收到导航触发信号: %s", msg.data)
    if not controller.is_navigating:
        controller.start_navigation()
    else:
        rospy.loginfo("导航正在进行中，忽略新请求")


if __name__ == "__main__":
    rospy.init_node('auto_slam_node', anonymous=True)
    controller = NavigationController()
    rospy.Subscriber("auto_slam", String, lambda msg: callback(msg, controller))
    rospy.loginfo("导航节点已启动，等待触发信号...")
    rospy.spin()




