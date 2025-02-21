; Auto-generated. Do not edit!


(cl:in-package roswifibot-msg)


;//! \htmlinclude IR.msg.html

(cl:defclass <IR> (roslisp-msg-protocol:ros-message)
  ((IR_front_left
    :reader IR_front_left
    :initarg :IR_front_left
    :type cl:float
    :initform 0.0)
   (IR_back_left
    :reader IR_back_left
    :initarg :IR_back_left
    :type cl:float
    :initform 0.0)
   (IR_front_right
    :reader IR_front_right
    :initarg :IR_front_right
    :type cl:float
    :initform 0.0)
   (IR_back_right
    :reader IR_back_right
    :initarg :IR_back_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass IR (<IR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roswifibot-msg:<IR> is deprecated: use roswifibot-msg:IR instead.")))

(cl:ensure-generic-function 'IR_front_left-val :lambda-list '(m))
(cl:defmethod IR_front_left-val ((m <IR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roswifibot-msg:IR_front_left-val is deprecated.  Use roswifibot-msg:IR_front_left instead.")
  (IR_front_left m))

(cl:ensure-generic-function 'IR_back_left-val :lambda-list '(m))
(cl:defmethod IR_back_left-val ((m <IR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roswifibot-msg:IR_back_left-val is deprecated.  Use roswifibot-msg:IR_back_left instead.")
  (IR_back_left m))

(cl:ensure-generic-function 'IR_front_right-val :lambda-list '(m))
(cl:defmethod IR_front_right-val ((m <IR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roswifibot-msg:IR_front_right-val is deprecated.  Use roswifibot-msg:IR_front_right instead.")
  (IR_front_right m))

(cl:ensure-generic-function 'IR_back_right-val :lambda-list '(m))
(cl:defmethod IR_back_right-val ((m <IR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roswifibot-msg:IR_back_right-val is deprecated.  Use roswifibot-msg:IR_back_right instead.")
  (IR_back_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IR>) ostream)
  "Serializes a message object of type '<IR>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR_front_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR_back_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR_front_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'IR_back_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IR>) istream)
  "Deserializes a message object of type '<IR>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR_front_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR_back_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR_front_right) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'IR_back_right) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IR>)))
  "Returns string type for a message object of type '<IR>"
  "roswifibot/IR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IR)))
  "Returns string type for a message object of type 'IR"
  "roswifibot/IR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IR>)))
  "Returns md5sum for a message object of type '<IR>"
  "08989c603acc510242caf5149106a2a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IR)))
  "Returns md5sum for a message object of type 'IR"
  "08989c603acc510242caf5149106a2a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IR>)))
  "Returns full string definition for message of type '<IR>"
  (cl:format cl:nil "float64 IR_front_left~%float64 IR_back_left~%float64 IR_front_right~%float64 IR_back_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IR)))
  "Returns full string definition for message of type 'IR"
  (cl:format cl:nil "float64 IR_front_left~%float64 IR_back_left~%float64 IR_front_right~%float64 IR_back_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IR>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IR>))
  "Converts a ROS message object to a list"
  (cl:list 'IR
    (cl:cons ':IR_front_left (IR_front_left msg))
    (cl:cons ':IR_back_left (IR_back_left msg))
    (cl:cons ':IR_front_right (IR_front_right msg))
    (cl:cons ':IR_back_right (IR_back_right msg))
))
