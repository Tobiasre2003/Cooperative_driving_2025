; Auto-generated. Do not edit!


(cl:in-package gv_client-msg)


;//! \htmlinclude LaptopSpeed.msg.html

(cl:defclass <LaptopSpeed> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tag_id
    :reader tag_id
    :initarg :tag_id
    :type cl:integer
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (restart
    :reader restart
    :initarg :restart
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LaptopSpeed (<LaptopSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LaptopSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LaptopSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gv_client-msg:<LaptopSpeed> is deprecated: use gv_client-msg:LaptopSpeed instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LaptopSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gv_client-msg:header-val is deprecated.  Use gv_client-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tag_id-val :lambda-list '(m))
(cl:defmethod tag_id-val ((m <LaptopSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gv_client-msg:tag_id-val is deprecated.  Use gv_client-msg:tag_id instead.")
  (tag_id m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <LaptopSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gv_client-msg:speed-val is deprecated.  Use gv_client-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'restart-val :lambda-list '(m))
(cl:defmethod restart-val ((m <LaptopSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gv_client-msg:restart-val is deprecated.  Use gv_client-msg:restart instead.")
  (restart m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LaptopSpeed>) ostream)
  "Serializes a message object of type '<LaptopSpeed>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'tag_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'restart) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LaptopSpeed>) istream)
  "Deserializes a message object of type '<LaptopSpeed>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'restart) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LaptopSpeed>)))
  "Returns string type for a message object of type '<LaptopSpeed>"
  "gv_client/LaptopSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LaptopSpeed)))
  "Returns string type for a message object of type 'LaptopSpeed"
  "gv_client/LaptopSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LaptopSpeed>)))
  "Returns md5sum for a message object of type '<LaptopSpeed>"
  "06654cff71f2995da6a11eca73ad520d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LaptopSpeed)))
  "Returns md5sum for a message object of type 'LaptopSpeed"
  "06654cff71f2995da6a11eca73ad520d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LaptopSpeed>)))
  "Returns full string definition for message of type '<LaptopSpeed>"
  (cl:format cl:nil "Header header~%int32 tag_id~%float32 speed~%bool restart~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LaptopSpeed)))
  "Returns full string definition for message of type 'LaptopSpeed"
  (cl:format cl:nil "Header header~%int32 tag_id~%float32 speed~%bool restart~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LaptopSpeed>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LaptopSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'LaptopSpeed
    (cl:cons ':header (header msg))
    (cl:cons ':tag_id (tag_id msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':restart (restart msg))
))
