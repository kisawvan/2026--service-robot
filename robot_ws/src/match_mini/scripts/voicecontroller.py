#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf8')
import os
import rospy

import time
# from respeaker_interface import RespeakerInterface
# from respeaker_audio import RespeakerAudio
from std_msgs.msg import String
import threading
import tty
import termios
import select

class VoiceController(object):
    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown)
        # self.respeaker_interface = RespeakerInterface()
        # self.respeaker_audio = RespeakerAudio()
        self.ask_pub = rospy.Publisher('cmd_msg', String, queue_size=5)
        self.key_pub = rospy.Publisher('key_input', String, queue_size=10)  # 新增按键发布器

    def shutdown(self):
        pass
        # self.respeaker_interface.close()
        # self.respeaker_audio.stop()

def callback(msg):
    os.system("mpg123 /home/bcsh/robot_ws/src/match_mini/voice/zhuabu.mp3")

# 键盘监听线程
def listen_keyboard():
    rospy.loginfo("按下 g、h 或 j 启动自动导航，按 q 退出...")
    
    if not sys.stdin.isatty():
        rospy.logwarn("没有检测到终端输入(可能因为使用了roslaunch无输出界面)，键盘监听功能将被禁用。")
        return

    # 保存原始终端设置
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        # 设置终端为单字符模式（无需回车）
        tty.setraw(sys.stdin.fileno())
        
        while not rospy.is_shutdown():
            # 检测按键按下
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                key_pub.publish(String(key))  # 发布按键信息
                
                if key == 'q':
                    rospy.signal_shutdown("用户退出")
                    break
                elif key in ['g', 'h', 'j']:
                    start_msg = String()
                    start_msg.data = "start"
                    auto_slam.publish(start_msg)
    finally:
        # 恢复原始终端设置
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    voice_controller = VoiceController("voice_controller")
    auto_slam = rospy.Publisher('auto_slam', String, queue_size=10)
    key_pub = rospy.Publisher('key_input', String, queue_size=10)  # 全局发布器
    rospy.Subscriber("get_pos", String, callback, queue_size=10)
    
    # 启动键盘监听线程
    keyboard_thread = threading.Thread(target=listen_keyboard)
    keyboard_thread.daemon = True
    keyboard_thread.start()
    
    rospy.spin()  # 进入ROS主循环
