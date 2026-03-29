#!/usr/bin/env python
import pyaudio
from baidu_speech_api import BaiduVoiceApi
import json
import sys
import os
from aip.speech import AipSpeech
from contextlib import contextmanager

reload(sys)
sys.setdefaultencoding( "utf-8" )

CHUNK = 1024
RECORD_SECONDS = 5
APP_ID = '55791428'
API_KEY = 'SKIJzVmbFWuwncLRaQJeng9v'
SECRET_KEY = 'nscrmppe0DkyKHvYlmN2xsgh1L4ZEWpY'

@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield
        
class RespeakerAudio(object):
    def __init__(self, channel=0, supress_error=True):
        with ignore_stderr(enable=supress_error):
            self.pyaudio = pyaudio.PyAudio()
        self.channels = None
        self.channel = channel
        self.device_index = None
        self.rate = 16000
        self.bitwidth = 2
        self.bitdepth = 16
        ####find device####
        count = self.pyaudio.get_device_count()
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            name = info["name"].encode("utf-8")
            chan = info["maxInputChannels"]
            if name.lower().find("respeaker") >= 0:
                self.channels = chan
                self.device_index = i
                break  
        if self.device_index is None:
            info = self.pyaudio.get_default_input_device_info()
            self.channels = info["maxInputChannels"]
            self.device_index = info["index"]
        self.channel = min(self.channels - 1, max(0, self.channel))
        self.stream = self.pyaudio.open(
            rate = self.rate,
            format=self.pyaudio.get_format_from_width(self.bitwidth),
            channels=1,
            input=True,
            input_device_index=self.device_index,
            )
        self.aipSpeech = AipSpeech(APP_ID, API_KEY, SECRET_KEY)
        self.baidu = BaiduVoiceApi(appkey=API_KEY,secretkey=SECRET_KEY)
    
    def stop(self):
		self.stop()
		try:
			self.stream.stop_stream()
			self.stream.close()
		except:
			pass
		finally:
			self.stream = None
		try:
			self.pyaudio.terminate()
		except:
			pass
 
    def generator_list(self, list):
        for l in list:
            yield l
            
    def record(self):
        self.stream.start_stream()
        print("* recording")
        frames = []
        for i in range(0, int(self.rate/CHUNK*RECORD_SECONDS)):
            data = self.stream.read(CHUNK)
            frames.append(data)
        print("done recording")
        self.stream.stop_stream()
        print("start to send to baidu")
        text = self.baidu.server_api(self.generator_list(frames))
        #print (text)
        if text:
            try:
                text = json.loads(text)
                for t in text['result']:
                    print(t)
                    #self.voice_publisher.publish(str(text))
                    return(str(t))
            except KeyError: 
                return("get nothing")
        else:
            print("get nothing")
            return("get nothing")
        
if __name__ == '__main__':
    snowman_audio = RespeakerAudio()
    while True:
        text = snowman_audio.record()
        #print (text)       
     
		
		
            
			
