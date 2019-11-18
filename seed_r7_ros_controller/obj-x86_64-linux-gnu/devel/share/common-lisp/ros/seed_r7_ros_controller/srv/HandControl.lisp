; Auto-generated. Do not edit!


(cl:in-package seed_r7_ros_controller-srv)


;//! \htmlinclude HandControl-request.msg.html

(cl:defclass <HandControl-request> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type cl:fixnum
    :initform 0)
   (script
    :reader script
    :initarg :script
    :type cl:string
    :initform "")
   (current
    :reader current
    :initarg :current
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HandControl-request (<HandControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name seed_r7_ros_controller-srv:<HandControl-request> is deprecated: use seed_r7_ros_controller-srv:HandControl-request instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <HandControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader seed_r7_ros_controller-srv:position-val is deprecated.  Use seed_r7_ros_controller-srv:position instead.")
  (position m))

(cl:ensure-generic-function 'script-val :lambda-list '(m))
(cl:defmethod script-val ((m <HandControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader seed_r7_ros_controller-srv:script-val is deprecated.  Use seed_r7_ros_controller-srv:script instead.")
  (script m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <HandControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader seed_r7_ros_controller-srv:current-val is deprecated.  Use seed_r7_ros_controller-srv:current instead.")
  (current m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HandControl-request>)))
    "Constants for message type '<HandControl-request>"
  '((:POSITION_RIGHT . 0)
    (:POSITION_LEFT . 1)
    (:SCRIPT_GRASP . grasp)
    (:SCRIPT_RELEASE . release)
    (:SCRIPT_CANCEL . cancel))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HandControl-request)))
    "Constants for message type 'HandControl-request"
  '((:POSITION_RIGHT . 0)
    (:POSITION_LEFT . 1)
    (:SCRIPT_GRASP . grasp)
    (:SCRIPT_RELEASE . release)
    (:SCRIPT_CANCEL . cancel))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandControl-request>) ostream)
  "Serializes a message object of type '<HandControl-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'script))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'script))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandControl-request>) istream)
  "Deserializes a message object of type '<HandControl-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'script) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'script) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandControl-request>)))
  "Returns string type for a service object of type '<HandControl-request>"
  "seed_r7_ros_controller/HandControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandControl-request)))
  "Returns string type for a service object of type 'HandControl-request"
  "seed_r7_ros_controller/HandControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandControl-request>)))
  "Returns md5sum for a message object of type '<HandControl-request>"
  "beb2871e68a142be80f3b23a0a3a73da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandControl-request)))
  "Returns md5sum for a message object of type 'HandControl-request"
  "beb2871e68a142be80f3b23a0a3a73da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandControl-request>)))
  "Returns full string definition for message of type '<HandControl-request>"
  (cl:format cl:nil "uint8 position~%uint8 POSITION_RIGHT = 0~%uint8 POSITION_LEFT = 1~%~%string script~%string SCRIPT_GRASP = grasp~%string SCRIPT_RELEASE = release~%string SCRIPT_CANCEL = cancel~%~%uint8  current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandControl-request)))
  "Returns full string definition for message of type 'HandControl-request"
  (cl:format cl:nil "uint8 position~%uint8 POSITION_RIGHT = 0~%uint8 POSITION_LEFT = 1~%~%string script~%string SCRIPT_GRASP = grasp~%string SCRIPT_RELEASE = release~%string SCRIPT_CANCEL = cancel~%~%uint8  current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandControl-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'script))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HandControl-request
    (cl:cons ':position (position msg))
    (cl:cons ':script (script msg))
    (cl:cons ':current (current msg))
))
;//! \htmlinclude HandControl-response.msg.html

(cl:defclass <HandControl-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass HandControl-response (<HandControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name seed_r7_ros_controller-srv:<HandControl-response> is deprecated: use seed_r7_ros_controller-srv:HandControl-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <HandControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader seed_r7_ros_controller-srv:result-val is deprecated.  Use seed_r7_ros_controller-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandControl-response>) ostream)
  "Serializes a message object of type '<HandControl-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandControl-response>) istream)
  "Deserializes a message object of type '<HandControl-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandControl-response>)))
  "Returns string type for a service object of type '<HandControl-response>"
  "seed_r7_ros_controller/HandControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandControl-response)))
  "Returns string type for a service object of type 'HandControl-response"
  "seed_r7_ros_controller/HandControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandControl-response>)))
  "Returns md5sum for a message object of type '<HandControl-response>"
  "beb2871e68a142be80f3b23a0a3a73da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandControl-response)))
  "Returns md5sum for a message object of type 'HandControl-response"
  "beb2871e68a142be80f3b23a0a3a73da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandControl-response>)))
  "Returns full string definition for message of type '<HandControl-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandControl-response)))
  "Returns full string definition for message of type 'HandControl-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandControl-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HandControl-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HandControl)))
  'HandControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HandControl)))
  'HandControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandControl)))
  "Returns string type for a service object of type '<HandControl>"
  "seed_r7_ros_controller/HandControl")