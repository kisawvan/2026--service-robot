
(cl:in-package :asdf)

(defsystem "robot_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AprilTagDetection" :depends-on ("_package_AprilTagDetection"))
    (:file "_package_AprilTagDetection" :depends-on ("_package"))
  ))