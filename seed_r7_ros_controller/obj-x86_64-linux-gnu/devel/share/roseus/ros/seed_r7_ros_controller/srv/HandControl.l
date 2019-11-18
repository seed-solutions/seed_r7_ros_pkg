;; Auto-generated. Do not edit!


(when (boundp 'seed_r7_ros_controller::HandControl)
  (if (not (find-package "SEED_R7_ROS_CONTROLLER"))
    (make-package "SEED_R7_ROS_CONTROLLER"))
  (shadow 'HandControl (find-package "SEED_R7_ROS_CONTROLLER")))
(unless (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROL")
  (make-package "SEED_R7_ROS_CONTROLLER::HANDCONTROL"))
(unless (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST")
  (make-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(unless (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLRESPONSE")
  (make-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLRESPONSE"))

(in-package "ROS")





(intern "*POSITION_RIGHT*" (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(shadow '*POSITION_RIGHT* (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(defconstant seed_r7_ros_controller::HandControlRequest::*POSITION_RIGHT* 0)
(intern "*POSITION_LEFT*" (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(shadow '*POSITION_LEFT* (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(defconstant seed_r7_ros_controller::HandControlRequest::*POSITION_LEFT* 1)
(intern "*SCRIPT_GRASP*" (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(shadow '*SCRIPT_GRASP* (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(defconstant seed_r7_ros_controller::HandControlRequest::*SCRIPT_GRASP* "grasp")
(intern "*SCRIPT_RELEASE*" (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(shadow '*SCRIPT_RELEASE* (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(defconstant seed_r7_ros_controller::HandControlRequest::*SCRIPT_RELEASE* "release")
(intern "*SCRIPT_CANCEL*" (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(shadow '*SCRIPT_CANCEL* (find-package "SEED_R7_ROS_CONTROLLER::HANDCONTROLREQUEST"))
(defconstant seed_r7_ros_controller::HandControlRequest::*SCRIPT_CANCEL* "cancel")
(defclass seed_r7_ros_controller::HandControlRequest
  :super ros::object
  :slots (_position _script _current ))

(defmethod seed_r7_ros_controller::HandControlRequest
  (:init
   (&key
    ((:position __position) 0)
    ((:script __script) "")
    ((:current __current) 0)
    )
   (send-super :init)
   (setq _position (round __position))
   (setq _script (string __script))
   (setq _current (round __current))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:script
   (&optional __script)
   (if __script (setq _script __script)) _script)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; uint8 _position
    1
    ;; string _script
    4 (length _script)
    ;; uint8 _current
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _position
       (write-byte _position s)
     ;; string _script
       (write-long (length _script) s) (princ _script s)
     ;; uint8 _current
       (write-byte _current s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _position
     (setq _position (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _script
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _script (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _current
     (setq _current (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass seed_r7_ros_controller::HandControlResponse
  :super ros::object
  :slots (_result ))

(defmethod seed_r7_ros_controller::HandControlResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass seed_r7_ros_controller::HandControl
  :super ros::object
  :slots ())

(setf (get seed_r7_ros_controller::HandControl :md5sum-) "beb2871e68a142be80f3b23a0a3a73da")
(setf (get seed_r7_ros_controller::HandControl :datatype-) "seed_r7_ros_controller/HandControl")
(setf (get seed_r7_ros_controller::HandControl :request) seed_r7_ros_controller::HandControlRequest)
(setf (get seed_r7_ros_controller::HandControl :response) seed_r7_ros_controller::HandControlResponse)

(defmethod seed_r7_ros_controller::HandControlRequest
  (:response () (instance seed_r7_ros_controller::HandControlResponse :init)))

(setf (get seed_r7_ros_controller::HandControlRequest :md5sum-) "beb2871e68a142be80f3b23a0a3a73da")
(setf (get seed_r7_ros_controller::HandControlRequest :datatype-) "seed_r7_ros_controller/HandControlRequest")
(setf (get seed_r7_ros_controller::HandControlRequest :definition-)
      "uint8 position
uint8 POSITION_RIGHT = 0
uint8 POSITION_LEFT = 1

string script
string SCRIPT_GRASP = grasp
string SCRIPT_RELEASE = release
string SCRIPT_CANCEL = cancel

uint8  current
---
string result

")

(setf (get seed_r7_ros_controller::HandControlResponse :md5sum-) "beb2871e68a142be80f3b23a0a3a73da")
(setf (get seed_r7_ros_controller::HandControlResponse :datatype-) "seed_r7_ros_controller/HandControlResponse")
(setf (get seed_r7_ros_controller::HandControlResponse :definition-)
      "uint8 position
uint8 POSITION_RIGHT = 0
uint8 POSITION_LEFT = 1

string script
string SCRIPT_GRASP = grasp
string SCRIPT_RELEASE = release
string SCRIPT_CANCEL = cancel

uint8  current
---
string result

")



(provide :seed_r7_ros_controller/HandControl "beb2871e68a142be80f3b23a0a3a73da")


