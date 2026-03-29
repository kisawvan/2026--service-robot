#!/usr/bin/env python
#coding:utf-8
from aip.speech import AipSpeech
import os
import rospy

#APP_ID = '11407471'
#API_KEY = 'GpsabveAZyggkRrNCMT1Ntr2'
#SECRET_KEY = 'm3XhjR3T9F3eM9257Euc4W1SfHZaddxh'


APP_ID = '21630733'
API_KEY = 'UQAbkY9cmbOHSx6GkIOWzyr2'
SECRET_KEY = 'WqTvStiFXXE7aFwvVrK1j4u9hjtxczio'

aipSpeech = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

string = ' 北京博创尚和科技有限公司是国家高新技术企业，核心技术团队由北航机器人研究所、中科院自动化所毕业的博士、硕士组成。深耕教育领域多年，以人工智能和机器人技术为核心，为各类高校和中小学提供智能机器人产品、实验室解决方案及课程服务，已与全国600余所高校和大量中小学在科研、教学、工程实践、机器人竞赛、创客空间建设等方面合作，得到了业内人士的普遍认可和好评。'
rospy.init_node('mini4_talk', anonymous=True)
result  = aipSpeech.synthesis(string, 'zh', 1, {
    'vol': 5, 'per': 5,
})


# 识别正确返回语音二进制 错误则返回dict 参照下面错误码
if not isinstance(result, dict):
    with open('faster.mp3', 'wb') as f:
        f.write(result)
    os.system("mpg123 faster.mp3")

