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
        self.last_tag_msg = None
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
        # 保存最近一条 AprilTag 消息
        try:
            self.last_tag_msg = msg
            self.tag_id = getattr(msg, 'tag_id', None)
        except Exception:
            self.last_tag_msg = None
            self.tag_id = None

        if self.waiting_for_tag:
            rospy.loginfo("检测到AprilTag ID: %s", str(self.tag_id))
            rospy.loginfo("AprilTag 消息: %s", str(self.last_tag_msg))
            self.waiting_for_tag = False

    def key_callback(self, msg):
        """处理按键信息回调"""
        self.key = msg.data  # 这里进行赋值
    "位置导航文件"
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

    def generate_special_path(self, start_x, start_y, idex, forward_length=0.75):
        """生成特殊弓字形路径点序列 (参考C++实现)"""
        # 固定侧向移动距离
        first_side_length = 0.15  # 15cm
        second_side_length = 0.12  # 20cm
        third_side_length = 0.12  # 20cm
        fourth_side_length = 0.15  # 15cm

        first_side_length2 = 0.15  # 15cm
        second_side_length2 = 0.12  # 20cm
        third_side_length2 = 0.12  # 20cm
        fourth_side_length2 = 0.15  # 15cm

        first_side_length3 = 0.15  # 15cm
        second_side_length3 = 0.12  # 20cm
        third_side_length3 = 0.12  # 20cm
        fourth_side_length3 = 0.15  # 15cm

        addition_side_length = 0.10 #15cm
        front_quat = tf_trans.quaternion_from_euler(0, 0, 0)  # 正0度
        back_quat = tf_trans.quaternion_from_euler(0, 0, math.pi)  # 反180度
        right_quat = tf_trans.quaternion_from_euler(0, 0, -math.pi / 2)  # 右90度
        left_quat = tf_trans.quaternion_from_euler(0, 0, math.pi / 2)  # 左90度
        
        if idex == 1:
            return [
                # 第一步: 前进forward_length (X方向)
                [start_x + forward_length, start_y, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第二步: 前进first_side_length (Y方向)
                [start_x + forward_length, start_y + first_side_length, 0.0, front_quat[0], front_quat[1], front_quat[2],
                front_quat[3]],

                # 第三步: 后退forward_length (负X方向)
                [start_x, start_y + first_side_length, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第四步: 前进second_side_length (Y方向)
                [start_x, start_y + first_side_length + second_side_length, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                # 第五步: 前进forward_length (X方向)
                [start_x + forward_length, start_y + first_side_length + second_side_length, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第六步: 前进forward_length (X方向)
                [start_x + forward_length, start_y + first_side_length + second_side_length + third_side_length, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                # 第七步: 前进forward_length (X方向)
                [start_x, start_y + first_side_length + second_side_length + third_side_length, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第八步: 前进forward_length (X方向)
                [start_x, start_y + first_side_length + second_side_length + third_side_length + fourth_side_length, 0.0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]],

                # 第九步: 前进forward_length (X方向)
                [start_x + forward_length,
                start_y + first_side_length + second_side_length + third_side_length + fourth_side_length, 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]]
            ]
        elif idex == 2:
            return [
                # 第一步: 前进forward_length (X方向)
                [start_x + forward_length, start_y, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第二步: 前进first_side_length (Y方向)
                [start_x + forward_length, start_y + first_side_length2, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                # 第三步: 后退forward_length (负X方向)
                [start_x, start_y + first_side_length2, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第四步: 前进second_side_length (Y方向)
                [start_x, start_y + first_side_length2 + second_side_length2, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                # 第五步: 前进forward_length (X方向)
                [start_x + forward_length, start_y + first_side_length2 + second_side_length2, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第六步: 前进forward_length (X方向)
                [start_x + forward_length, start_y + first_side_length2 + second_side_length2 + third_side_length2, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                # 第七步: 前进forward_length (X方向)
                [start_x, start_y + first_side_length2 + second_side_length2 + third_side_length2, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第八步: 前进forward_length (X方向)
                [start_x, start_y + first_side_length2 + second_side_length2 + third_side_length2 + fourth_side_length2, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                # 第九步: 前进forward_length (X方向)
                [start_x + forward_length,
                start_y + first_side_length2 + second_side_length2 + third_side_length2 + fourth_side_length2, 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]]
	]
        elif idex == 3:
            return [
                # 第一步: 前进forward_length (X方向)
                [start_x + forward_length, start_y, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第二步: 前进first_side_length (Y方向)
                [start_x + forward_length, start_y + first_side_length3 + addition_side_length, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                # 第三步: 后退forward_length (负X方向)
                [start_x, start_y + first_side_length3 + addition_side_length, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第四步: 前进second_side_length (Y方向)
                [start_x, start_y + first_side_length3 + second_side_length3 + addition_side_length, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                # 第五步: 前进forward_length (X方向)
                [start_x + forward_length, start_y + first_side_length3 + second_side_length3 + addition_side_length, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第六步: 前进forward_length (X方向)
                [start_x + forward_length, start_y + first_side_length3 + second_side_length3 + third_side_length3 + addition_side_length, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                # 第七步: 前进forward_length (X方向)
                [start_x, start_y + first_side_length3 + second_side_length3 + third_side_length3 + addition_side_length, 0.0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]],

                # 第八步: 前进forward_length (X方向)
                [start_x, start_y + first_side_length3 + second_side_length3 + third_side_length3 + fourth_side_length3 + addition_side_length, 0.0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]],

                # 第九步: 前进forward_length (X方向)
                [start_x + forward_length,
                start_y + first_side_length3 + second_side_length3 + third_side_length3 + fourth_side_length3 + addition_side_length, 0.0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]]
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
        # 获取路径点
        if idex == 1:
            path_points = self.generate_special_path(start_x, start_y, 1)
        elif idex == 2:
            path_points = self.generate_special_path(start_x, start_y, 2)
        elif idex == 3:
            path_points = self.generate_special_path(start_x, start_y, 3, 0.8)

        # 执行所有路径点
        for i, point in enumerate(path_points):
            if not self.navigate_to_point(point, i + 1, len(path_points) + 1):
                rospy.logwarn("路径点 %d 导航失败", i + 1)
                return False
        rospy.sleep(0.04)

        return True

    def navigate_points(self, points, id):
        """导航主航点并在关键点执行区域覆盖"""
        front_quat = tf_trans.quaternion_from_euler(0, 0, 0)  # 正0度
        back_quat = tf_trans.quaternion_from_euler(0, 0, math.pi)  # 反180度
        right_quat = tf_trans.quaternion_from_euler(0, 0, -math.pi / 2)  # 右90度
        left_quat = tf_trans.quaternion_from_euler(0, 0, math.pi / 2)  # 左90度

        if id == 1:
            self.navigate_to_point([1.32, -0.12, 0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]], 1, 3)  # 避免撞箱子
            self.navigate_to_point(points[0], 1, 3)  # 第一个区域
            rospy.sleep(0.05)
            self.execute_special_path(points[0][0], points[0][1], 1)
            rospy.sleep(0.05)

            self.navigate_to_point([2.4, 1.4, 0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]], 1, 3)  # 避免撞箱子
	    self.navigate_to_point([2.57, 2.4, 0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]], 1, 3)
            self.navigate_to_point(points[1], 2, 3)  # 第二个区域
            rospy.sleep(0.05)
            self.execute_special_path(points[1][0], points[1][1], 1)
            rospy.sleep(0.05)

            self.navigate_to_point([1.4, 2.55, 0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]], 1, 3)  # 避免撞箱子
            self.navigate_to_point(points[2], 3, 3)  # 第三个区域
	    rospy.sleep(0.05)
            self.execute_special_path(points[2][0], points[2][1], 2)
            rospy.sleep(0.05)
    	    self.navigate_to_point([0.12, 2.40, 0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]], 1, 3)
            self.navigate_to_point([0.12, 1.40, 0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]], 1, 3)


        elif id == 3:
            self.navigate_to_point([0.1, 0.25, 0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]], 1, 3)
            self.navigate_to_point([0.1, 1.5, 0, left_quat[0], left_quat[1], left_quat[2], left_quat[3]], 1, 3)
            self.navigate_to_point(points[0], 1, 3)  # 第一个区域
            rospy.sleep(0.05)
            self.execute_special_path(points[0][0], points[0][1], 3)
            rospy.sleep(0.05)
            self.navigate_to_point([1.19, 2.46, 0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]], 1, 3) # 避免撞箱子           
	    self.navigate_to_point([1.5, 2.46, 0, front_quat[0], front_quat[1], front_quat[2], front_quat[3]], 1, 3) 
	    self.navigate_to_point(points[1], 2, 3)  # 第二个区域
            rospy.sleep(0.05)
            self.execute_special_path(points[1][0], points[1][1], 1)
            rospy.sleep(0.05)

            self.navigate_to_point([2.53, 1.51, 0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]], 1, 3)  # 定点
	    self.navigate_to_point([2.53, 0.1, 0, right_quat[0], right_quat[1], right_quat[2], right_quat[3]], 1, 3)
            self.navigate_to_point(points[2], 3, 3)  # 第三个区域
            rospy.sleep(0.05)
            self.execute_special_path(points[2][0], points[2][1], 1)
            rospy.sleep(0.05)
 	    self.navigate_to_point([2.0, 0.0, 0, back_quat[0], back_quat[1], back_quat[2], back_quat[3]], 1, 3)

    def start_navigation(self):
        if self.is_navigating:
            rospy.logwarn("导航正在进行中，忽略新请求")
            return

        self.is_navigating = True
        rospy.loginfo("开始导航流程")
        
        # 修改流程：不再在起点进行二维码检测。
        # 流程：根据按键选择路径 -> 先执行前往并在 区域2 做前进动作 -> 直接进行弓字形清扫 -> 在清扫完成点等待二维码识别
        xml_file = ""
        key_val = getattr(self, 'key', None)
        if key_val == "g":
            xml_file = "/home/bcsh/waypoints5_1.xml"
        elif key_val == "h":
            xml_file = "/home/bcsh/waypoints5_1.xml"
            self.tag_id = 1
        elif key_val == "j":
            xml_file = "/home/bcsh/waypoints5_3.xml"
            self.tag_id = 3
        else:
            xml_file = "/home/bcsh/waypoints5_1.xml"

        # 解析并执行路径（先到区域2，再进行弓字形清扫，然后在清扫结束点等待二维码识别）
        points = self.parse_xml(xml_file)
        if points:
            # 如果按下 'g'，优先前往 (1.9, 1.87)
            if key_val == "g":
                try:
                    init_quat = tf_trans.quaternion_from_euler(0, 0, 0)
                    init_point = [1.9, 1.87, 0, init_quat[0], init_quat[1], init_quat[2], init_quat[3]]
                    rospy.loginfo("按下'g'，先前往初始化点 (1.9, 1.87)")
                    self.navigate_to_point(init_point, 0, len(points))
                    rospy.sleep(0.05)
                except Exception as e:
                    rospy.logwarn("前往初始化点失败: %s", str(e))
            # 先前往区域2（points[1]）并执行前进动作
            try:
                rospy.loginfo("前往区域2并执行前进动作")
                self.navigate_to_point(points[1], 1, len(points))
                rospy.sleep(0.05)
            except Exception as e:
                rospy.logwarn("前往区域2失败: %s", str(e))

            # 直接开始弓字形清扫（以区域2为起点）
            rospy.loginfo("开始弓字形清扫（区域2 起点）")
            self.execute_special_path(points[1][0], points[1][1], 1)
            rospy.sleep(0.05)

            # 区域2清扫完成后，前往指定点 (1.89, 2.17) 并设定朝向 45度
            try:
                target_quat = tf_trans.quaternion_from_euler(0, 0, math.pi / 4)
                target_point = [1.89, 2.17, 0, target_quat[0], target_quat[1], target_quat[2], target_quat[3]]
                rospy.loginfo("清扫完成，前往检测点 (1.89, 2.17) 并设朝向 45°")
                self.navigate_to_point(target_point, 0, len(points))
                rospy.sleep(0.05)
            except Exception as e:
                rospy.logwarn("前往检测点失败: %s", str(e))

            # 到达后开启 AprilTag 识别等待（无限等待直到检测到二维码或节点关闭）
            rospy.loginfo("开启 AprilTag 识别并等待结果...")
            self.waiting_for_tag = True
            self.tag_id = None
            self.last_tag_msg = None
            while (not rospy.is_shutdown() and self.waiting_for_tag):
                rospy.sleep(0.1)

            # 打印识别到的结果
            rospy.loginfo("AprilTag 识别完成，tag_id=%s", str(self.tag_id))
            rospy.loginfo("AprilTag 原始消息：%s", str(self.last_tag_msg))

            # 打印当前路径点1和路径点2坐标（如果存在）
            try:
                if len(points) >= 2:
                    rospy.loginfo("路径点1 坐标: x=%.3f, y=%.3f", points[0][0], points[0][1])
                    rospy.loginfo("路径点2 坐标: x=%.3f, y=%.3f", points[1][0], points[1][1])
                else:
                    rospy.logwarn("路径点数量不足，无法打印路径点1/2坐标")
            except Exception as e:
                rospy.logwarn("打印路径点坐标出错: %s", str(e))

            # 根据识别结果决定执行顺序：
            # tag==1 -> 先执行 points[0] 的弓字形，再执行 points[2] 的弓字形
            # tag==3 -> 先执行 points[2] 的弓字形，再执行 points[0] 的弓字形
            try:
                tag = self.tag_id if self.tag_id is not None else 1
                # 需要至少 3 个路径点来执行点1和点3
                if len(points) < 3:
                    rospy.logwarn("导航点数量不足（%d），无法按要求执行点1/点3的弓字形", len(points))
                else:
                    if tag == 1:
                        rospy.loginfo("Tag==1: 先执行路径点1的弓字形，再执行路径点3的弓字形")
                        self.navigate_to_point(points[0], 1, len(points))
                        rospy.sleep(0.05)
                        self.execute_special_path(points[0][0], points[0][1], 1)
                        rospy.sleep(0.05)

                        self.navigate_to_point(points[2], 2, len(points))
                        rospy.sleep(0.05)
                        self.execute_special_path(points[2][0], points[2][1], 2)
                        rospy.sleep(0.05)

                    elif tag == 3:
                        rospy.loginfo("Tag==3: 先执行路径点3的弓字形，再执行路径点1的弓字形")
                        self.navigate_to_point(points[2], 1, len(points))
                        rospy.sleep(0.05)
                        self.execute_special_path(points[2][0], points[2][1], 1)
                        rospy.sleep(0.05)

                        self.navigate_to_point(points[0], 2, len(points))
                        rospy.sleep(0.05)
                        self.execute_special_path(points[0][0], points[0][1], 1)
                        rospy.sleep(0.05)

                    else:
                        rospy.loginfo("未识别到指定 Tag（%s），使用默认导航逻辑", str(tag))
                        self.navigate_points(points, tag)

                # 完成后：优先返回路径点4(points[3])并在该点停止；若不存在则返回原点 (0,0)
                try:
                    if len(points) > 3:
                        rospy.loginfo("任务完成，返回路径点4 并在该点停止运行")
                        return_point = points[3]
                        success = self.navigate_to_point(return_point, 0, 1)
                        if success:
                            try:
                                self.cmd_vel_pub.publish(Twist())
                            except Exception as e:
                                rospy.logwarn("停止机器人时出错: %s", str(e))
                            self.is_navigating = False
                            rospy.loginfo("已到达路径点4并停止运行")
                            return
                        else:
                            rospy.logwarn("到达路径点4失败，尝试停止并结束流程")
                            try:
                                self.cmd_vel_pub.publish(Twist())
                            except Exception:
                                pass
                            self.is_navigating = False
                            return
                    else:
                        origin_quat = tf_trans.quaternion_from_euler(0, 0, 0)
                        origin_point = [0.0, 0.0, 0.0, origin_quat[0], origin_quat[1], origin_quat[2], origin_quat[3]]
                        rospy.loginfo("任务完成，路径点4不存在，返回起点 (0,0)")
                        success = self.navigate_to_point(origin_point, 0, 1)
                        if success:
                            try:
                                self.cmd_vel_pub.publish(Twist())
                            except Exception as e:
                                rospy.logwarn("停止机器人时出错: %s", str(e))
                        self.is_navigating = False
                        rospy.loginfo("导航流程结束")
                        return

                except Exception as e:
                    rospy.logwarn("返回终点失败: %s", str(e))
                    self.is_navigating = False
                    return
            except Exception as e:
                rospy.logwarn("根据 Tag 执行后续路径时出错: %s", str(e))
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





