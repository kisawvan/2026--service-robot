#!/usr/bin/env python
#coding:utf-8
from aip.speech import AipSpeech
import os
import rospy
from std_msgs.msg import String

APP_ID = '31643693'
API_KEY = '6PxVlGs0svhe3sIFQHXeRNI5'
SECRET_KEY = 'fjOWdcBLWxWdGrNgrUtH18uHa2VpSoXn'
aipSpeech = AipSpeech(APP_ID, API_KEY, SECRET_KEY)


def callback(data):
    text = data.data
    result  = aipSpeech.synthesis(text, 'zh', 1, {
        'vol': 5, 'per': 5,
    })
    # 识别正确返回语音二进制 错误则返回dict 参照下面错误码
    if not isinstance(result, dict):
        with open('faster.mp3', 'wb') as f:
            f.write(result)
        os.system("mpg123 faster.mp3")

def listener():
    rospy.init_node('snowmananswer')
    rospy.Subscriber('/snowman/answer', String, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()

