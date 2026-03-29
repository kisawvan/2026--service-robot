#!/usr/bin/env python

import rospy
import os
import sys
import cv2
import numpy as np
from ros_opencv import ROS2OPENCV

def read_images(path, sz=None):
    c = 0
    X, y = [], []
    names = []
    for dirname, dirnames, filenames in os.walk(path):
        for subdirname in dirnames:
            subject_path = os.path.join(dirname, subdirname)
            for filename in os.listdir(subject_path):
                try:
                    if (filename == ".directory"):
                        continue
                    filepath = os.path.join(subject_path, filename)
                    im = cv2.imread(os.path.join(subject_path, filename), cv2.IMREAD_GRAYSCALE)
                    if (im is None):
                        print("image" + filepath + "is None")
                    if (sz is not None):
                        im = cv2.resize(im, sz)
                    X.append(np.asarray(im, dtype=np.uint8))
                    y.append(c)
                except:
                    print("unexpected error")
                    raise
            c = c+1
            names.append(subdirname)
    return [names, X, y]

class FaceRecognizer(ROS2OPENCV):
    def __init__(self, node_name):
        super(FaceRecognizer, self).__init__(node_name)
        self.detect_box = None
        self.result = None
        self.names = None
        self.X = None
        self.Y = None
        self.face_cascade = cv2.CascadeClassifier('/home/bcsh/vision_ws/src/robot_vision/scripts/cascades/haarcascade_frontalface_default.xml')
        self.read_dir = "/home/bcsh/vision_ws/src/robot_vision/scripts/data"
        [names, X, Y] = read_images(self.read_dir)
        Y = np.asarray(Y, dtype=np.int32)
        self.names = names
        self.X = X
        self.Y = Y
        #self.model = cv2.face_EigenFaceRecognizer.create()
        self.model = cv2.face.LBPHFaceRecognizer_create()
        self.model.train(np.asarray(X), np.asarray(Y))
      
    def process_image(self, frame):
       src = frame.copy()
       gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
       faces = self.face_cascade.detectMultiScale(gray, 1.2, 3)
       result = src.copy()
       self.result = result
       for (x, y, w, h) in faces:
           result = cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
           roi = gray[y:y+h,x:x+w ]
           try:
               roi = cv2.resize(roi, (200,200), interpolation=cv2.INTER_LINEAR)
               [p_label, p_confidence] = self.model.predict(roi)
	       print"confidence: %s"%(p_confidence)  
	       if p_confidence<50:
                   cv2.putText(result, self.names[p_label], (x, y-20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
           except:
               continue 
       return result          
		            
		
	
'''
def face_rec():
    read_dir = "/home/bcsh/vision_ws/src/robot_vision/scripts/person1"
    [names, X, y] = read_images(read_dir)
    y = np.asarray(y, dtype=np.int32)
    model = cv2.face_EigenFaceRecognizer.create()
    model.train(np.asarray(X), np.asarray(y))
    
    face_cascade = cv2.CascadeClassifier('/home/bcsh/vision_ws/src/robot_vision/scripts/cascades/haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        x, y = frame.shape[0:2]
        small_frame = cv2.resize(frame, (int(y/2), int(x/2)))
        result = small_frame.copy()
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in faces:
            result = cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
            roi = gray[x:x+w, y:y+h]
            try:
                roi = cv2.resize(roi, (200,200), interpolation=cv2.INTER_LINEAR)
                [p_label, p_confidence] = model.predict(roi)
		print"confidence: %s"%(p_confidence)  
		if(p_confidence>9500):
                	cv2.putText(result, names[p_label], (x, y-20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            except:
                continue
        cv2.imshow("recognize_face", result)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
'''
   
if __name__ == "__main__":
    try:
        node_name = "face_recognizer"
        FaceRecognizer(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
cv2.destroyAllWindows()










