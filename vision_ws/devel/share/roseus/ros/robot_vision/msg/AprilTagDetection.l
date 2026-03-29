;; Auto-generated. Do not edit!


(when (boundp 'robot_vision::AprilTagDetection)
  (if (not (find-package "ROBOT_VISION"))
    (make-package "ROBOT_VISION"))
  (shadow 'AprilTagDetection (find-package "ROBOT_VISION")))
(unless (find-package "ROBOT_VISION::APRILTAGDETECTION")
  (make-package "ROBOT_VISION::APRILTAGDETECTION"))

(in-package "ROS")
;;//! \htmlinclude AprilTagDetection.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robot_vision::AprilTagDetection
  :super ros::object
  :slots (_header _tag_id _x _y ))

(defmethod robot_vision::AprilTagDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:tag_id __tag_id) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _tag_id (round __tag_id))
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:tag_id
   (&optional __tag_id)
   (if __tag_id (setq _tag_id __tag_id)) _tag_id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _tag_id
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _tag_id
       (write-long _tag_id s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _tag_id
     (setq _tag_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_vision::AprilTagDetection :md5sum-) "e1cc25b9cb52d5e2c3dce399058fac83")
(setf (get robot_vision::AprilTagDetection :datatype-) "robot_vision/AprilTagDetection")
(setf (get robot_vision::AprilTagDetection :definition-)
      "std_msgs/Header header
int32 tag_id
float32 x
float32 y

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :robot_vision/AprilTagDetection "e1cc25b9cb52d5e2c3dce399058fac83")


