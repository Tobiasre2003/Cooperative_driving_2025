; Auto-generated. Do not edit!


(cl:in-package mission_planner-msg)


;//! \htmlinclude Reservation.msg.html

(cl:defclass <Reservation> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:string
    :initform "")
   (end
    :reader end
    :initarg :end
    :type cl:string
    :initform ""))
)

(cl:defclass Reservation (<Reservation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Reservation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Reservation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_planner-msg:<Reservation> is deprecated: use mission_planner-msg:Reservation instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <Reservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_planner-msg:start-val is deprecated.  Use mission_planner-msg:start instead.")
  (start m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <Reservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_planner-msg:end-val is deprecated.  Use mission_planner-msg:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Reservation>) ostream)
  "Serializes a message object of type '<Reservation>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'start))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'end))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'end))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Reservation>) istream)
  "Deserializes a message object of type '<Reservation>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'start) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'end) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'end) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Reservation>)))
  "Returns string type for a message object of type '<Reservation>"
  "mission_planner/Reservation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reservation)))
  "Returns string type for a message object of type 'Reservation"
  "mission_planner/Reservation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Reservation>)))
  "Returns md5sum for a message object of type '<Reservation>"
  "fd9b411e8e594457e56809ca789fc69e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Reservation)))
  "Returns md5sum for a message object of type 'Reservation"
  "fd9b411e8e594457e56809ca789fc69e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Reservation>)))
  "Returns full string definition for message of type '<Reservation>"
  (cl:format cl:nil "string start~%string end~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Reservation)))
  "Returns full string definition for message of type 'Reservation"
  (cl:format cl:nil "string start~%string end~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Reservation>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'start))
     4 (cl:length (cl:slot-value msg 'end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Reservation>))
  "Converts a ROS message object to a list"
  (cl:list 'Reservation
    (cl:cons ':start (start msg))
    (cl:cons ':end (end msg))
))
