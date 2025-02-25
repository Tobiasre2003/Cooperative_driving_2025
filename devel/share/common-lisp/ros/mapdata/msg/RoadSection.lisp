; Auto-generated. Do not edit!


(cl:in-package mapdata-msg)


;//! \htmlinclude RoadSection.msg.html

(cl:defclass <RoadSection> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type mapdata-msg:Position
    :initform (cl:make-instance 'mapdata-msg:Position))
   (right
    :reader right
    :initarg :right
    :type mapdata-msg:Position
    :initform (cl:make-instance 'mapdata-msg:Position))
   (length
    :reader length
    :initarg :length
    :type cl:integer
    :initform 0)
   (stopline_offset
    :reader stopline_offset
    :initarg :stopline_offset
    :type cl:integer
    :initform 0)
   (priority_sign
    :reader priority_sign
    :initarg :priority_sign
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass RoadSection (<RoadSection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoadSection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoadSection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapdata-msg:<RoadSection> is deprecated: use mapdata-msg:RoadSection instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <RoadSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:left-val is deprecated.  Use mapdata-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <RoadSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:right-val is deprecated.  Use mapdata-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <RoadSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:length-val is deprecated.  Use mapdata-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'stopline_offset-val :lambda-list '(m))
(cl:defmethod stopline_offset-val ((m <RoadSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:stopline_offset-val is deprecated.  Use mapdata-msg:stopline_offset instead.")
  (stopline_offset m))

(cl:ensure-generic-function 'priority_sign-val :lambda-list '(m))
(cl:defmethod priority_sign-val ((m <RoadSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:priority_sign-val is deprecated.  Use mapdata-msg:priority_sign instead.")
  (priority_sign m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <RoadSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:name-val is deprecated.  Use mapdata-msg:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RoadSection>)))
    "Constants for message type '<RoadSection>"
  '((:PRIORITY_ROAD . 0)
    (:GIVE_WAY . 1)
    (:STOP_SIGN . 2)
    (:TRAFFIC_LIGHT . 3)
    (:BOOKING . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RoadSection)))
    "Constants for message type 'RoadSection"
  '((:PRIORITY_ROAD . 0)
    (:GIVE_WAY . 1)
    (:STOP_SIGN . 2)
    (:TRAFFIC_LIGHT . 3)
    (:BOOKING . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoadSection>) ostream)
  "Serializes a message object of type '<RoadSection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right) ostream)
  (cl:let* ((signed (cl:slot-value msg 'length)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stopline_offset)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'priority_sign)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoadSection>) istream)
  "Deserializes a message object of type '<RoadSection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'length) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stopline_offset) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'priority_sign)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoadSection>)))
  "Returns string type for a message object of type '<RoadSection>"
  "mapdata/RoadSection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoadSection)))
  "Returns string type for a message object of type 'RoadSection"
  "mapdata/RoadSection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoadSection>)))
  "Returns md5sum for a message object of type '<RoadSection>"
  "e564107726b89210b4492752f37581d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoadSection)))
  "Returns md5sum for a message object of type 'RoadSection"
  "e564107726b89210b4492752f37581d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoadSection>)))
  "Returns full string definition for message of type '<RoadSection>"
  (cl:format cl:nil "# If you're standing on the road looking towards the intersection, the left~%# position is the leftmost edge of the rectangle closest to the intersection.~%#    |       |~%#    |       |~%#  right    left~%# ---+       +----~%#~%Position left~%Position right~%~%# How far the road section extends from the intersection~%int32 length~%~%# How far from the intersection the cars should stop~%int32 stopline_offset~%~%# Enumeration (just constants) of priority signs~%uint8 PRIORITY_ROAD=0~%uint8 GIVE_WAY=1~%uint8 STOP_SIGN=2~%uint8 TRAFFIC_LIGHT=3~%uint8 BOOKING=4~%~%uint8 priority_sign~%~%# A bit redundant but nice for pretty printing~%string name~%~%================================================================================~%MSG: mapdata/Position~%int32 x~%int32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoadSection)))
  "Returns full string definition for message of type 'RoadSection"
  (cl:format cl:nil "# If you're standing on the road looking towards the intersection, the left~%# position is the leftmost edge of the rectangle closest to the intersection.~%#    |       |~%#    |       |~%#  right    left~%# ---+       +----~%#~%Position left~%Position right~%~%# How far the road section extends from the intersection~%int32 length~%~%# How far from the intersection the cars should stop~%int32 stopline_offset~%~%# Enumeration (just constants) of priority signs~%uint8 PRIORITY_ROAD=0~%uint8 GIVE_WAY=1~%uint8 STOP_SIGN=2~%uint8 TRAFFIC_LIGHT=3~%uint8 BOOKING=4~%~%uint8 priority_sign~%~%# A bit redundant but nice for pretty printing~%string name~%~%================================================================================~%MSG: mapdata/Position~%int32 x~%int32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoadSection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right))
     4
     4
     1
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoadSection>))
  "Converts a ROS message object to a list"
  (cl:list 'RoadSection
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
    (cl:cons ':length (length msg))
    (cl:cons ':stopline_offset (stopline_offset msg))
    (cl:cons ':priority_sign (priority_sign msg))
    (cl:cons ':name (name msg))
))
