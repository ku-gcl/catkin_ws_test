; Auto-generated. Do not edit!


(cl:in-package test-msg)


;//! \htmlinclude CustomState.msg.html

(cl:defclass <CustomState> (roslisp-msg-protocol:ros-message)
  ((theta1
    :reader theta1
    :initarg :theta1
    :type cl:float
    :initform 0.0)
   (theta1_dot
    :reader theta1_dot
    :initarg :theta1_dot
    :type cl:float
    :initform 0.0)
   (theta2
    :reader theta2
    :initarg :theta2
    :type cl:float
    :initform 0.0)
   (theta2_dot
    :reader theta2_dot
    :initarg :theta2_dot
    :type cl:float
    :initform 0.0))
)

(cl:defclass CustomState (<CustomState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CustomState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CustomState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name test-msg:<CustomState> is deprecated: use test-msg:CustomState instead.")))

(cl:ensure-generic-function 'theta1-val :lambda-list '(m))
(cl:defmethod theta1-val ((m <CustomState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test-msg:theta1-val is deprecated.  Use test-msg:theta1 instead.")
  (theta1 m))

(cl:ensure-generic-function 'theta1_dot-val :lambda-list '(m))
(cl:defmethod theta1_dot-val ((m <CustomState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test-msg:theta1_dot-val is deprecated.  Use test-msg:theta1_dot instead.")
  (theta1_dot m))

(cl:ensure-generic-function 'theta2-val :lambda-list '(m))
(cl:defmethod theta2-val ((m <CustomState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test-msg:theta2-val is deprecated.  Use test-msg:theta2 instead.")
  (theta2 m))

(cl:ensure-generic-function 'theta2_dot-val :lambda-list '(m))
(cl:defmethod theta2_dot-val ((m <CustomState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test-msg:theta2_dot-val is deprecated.  Use test-msg:theta2_dot instead.")
  (theta2_dot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CustomState>) ostream)
  "Serializes a message object of type '<CustomState>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta1_dot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta2_dot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CustomState>) istream)
  "Deserializes a message object of type '<CustomState>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta1_dot) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta2_dot) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CustomState>)))
  "Returns string type for a message object of type '<CustomState>"
  "test/CustomState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomState)))
  "Returns string type for a message object of type 'CustomState"
  "test/CustomState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CustomState>)))
  "Returns md5sum for a message object of type '<CustomState>"
  "a8d2dcedf46285bfb75a4965f00f0338")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CustomState)))
  "Returns md5sum for a message object of type 'CustomState"
  "a8d2dcedf46285bfb75a4965f00f0338")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CustomState>)))
  "Returns full string definition for message of type '<CustomState>"
  (cl:format cl:nil "float64 theta1~%float64 theta1_dot~%float64 theta2~%float64 theta2_dot~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CustomState)))
  "Returns full string definition for message of type 'CustomState"
  (cl:format cl:nil "float64 theta1~%float64 theta1_dot~%float64 theta2~%float64 theta2_dot~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CustomState>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CustomState>))
  "Converts a ROS message object to a list"
  (cl:list 'CustomState
    (cl:cons ':theta1 (theta1 msg))
    (cl:cons ':theta1_dot (theta1_dot msg))
    (cl:cons ':theta2 (theta2 msg))
    (cl:cons ':theta2_dot (theta2_dot msg))
))
