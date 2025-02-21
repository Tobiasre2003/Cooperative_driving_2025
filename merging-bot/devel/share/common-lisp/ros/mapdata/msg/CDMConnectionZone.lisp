; Auto-generated. Do not edit!


(cl:in-package mapdata-msg)


;//! \htmlinclude CDMConnectionZone.msg.html

(cl:defclass <CDMConnectionZone> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type mapdata-msg:Position
    :initform (cl:make-instance 'mapdata-msg:Position))
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0))
)

(cl:defclass CDMConnectionZone (<CDMConnectionZone>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CDMConnectionZone>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CDMConnectionZone)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapdata-msg:<CDMConnectionZone> is deprecated: use mapdata-msg:CDMConnectionZone instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <CDMConnectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:pos-val is deprecated.  Use mapdata-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <CDMConnectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:width-val is deprecated.  Use mapdata-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <CDMConnectionZone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:height-val is deprecated.  Use mapdata-msg:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CDMConnectionZone>) ostream)
  "Serializes a message object of type '<CDMConnectionZone>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CDMConnectionZone>) istream)
  "Deserializes a message object of type '<CDMConnectionZone>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CDMConnectionZone>)))
  "Returns string type for a message object of type '<CDMConnectionZone>"
  "mapdata/CDMConnectionZone")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CDMConnectionZone)))
  "Returns string type for a message object of type 'CDMConnectionZone"
  "mapdata/CDMConnectionZone")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CDMConnectionZone>)))
  "Returns md5sum for a message object of type '<CDMConnectionZone>"
  "49ee6d673b89f273a2281d8ddb4755f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CDMConnectionZone)))
  "Returns md5sum for a message object of type 'CDMConnectionZone"
  "49ee6d673b89f273a2281d8ddb4755f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CDMConnectionZone>)))
  "Returns full string definition for message of type '<CDMConnectionZone>"
  (cl:format cl:nil "Position pos~%uint32 width~%uint32 height~%~%================================================================================~%MSG: mapdata/Position~%int32 x~%int32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CDMConnectionZone)))
  "Returns full string definition for message of type 'CDMConnectionZone"
  (cl:format cl:nil "Position pos~%uint32 width~%uint32 height~%~%================================================================================~%MSG: mapdata/Position~%int32 x~%int32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CDMConnectionZone>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CDMConnectionZone>))
  "Converts a ROS message object to a list"
  (cl:list 'CDMConnectionZone
    (cl:cons ':pos (pos msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
