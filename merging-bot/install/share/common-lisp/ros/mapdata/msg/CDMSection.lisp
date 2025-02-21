; Auto-generated. Do not edit!


(cl:in-package mapdata-msg)


;//! \htmlinclude CDMSection.msg.html

(cl:defclass <CDMSection> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (pos
    :reader pos
    :initarg :pos
    :type mapdata-msg:Position
    :initform (cl:make-instance 'mapdata-msg:Position))
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (orientation
    :reader orientation
    :initarg :orientation
    :type cl:integer
    :initform 0)
   (connection_zones
    :reader connection_zones
    :initarg :connection_zones
    :type (cl:vector mapdata-msg:CDMConnectionZone)
   :initform (cl:make-array 0 :element-type 'mapdata-msg:CDMConnectionZone :initial-element (cl:make-instance 'mapdata-msg:CDMConnectionZone))))
)

(cl:defclass CDMSection (<CDMSection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CDMSection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CDMSection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapdata-msg:<CDMSection> is deprecated: use mapdata-msg:CDMSection instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <CDMSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:id-val is deprecated.  Use mapdata-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <CDMSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:pos-val is deprecated.  Use mapdata-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <CDMSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:type-val is deprecated.  Use mapdata-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <CDMSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:width-val is deprecated.  Use mapdata-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <CDMSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:height-val is deprecated.  Use mapdata-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <CDMSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:orientation-val is deprecated.  Use mapdata-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'connection_zones-val :lambda-list '(m))
(cl:defmethod connection_zones-val ((m <CDMSection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-msg:connection_zones-val is deprecated.  Use mapdata-msg:connection_zones instead.")
  (connection_zones m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CDMSection>)))
    "Constants for message type '<CDMSection>"
  '((:THREE_WAY_INTERSECTION . 0)
    (:FOUR_WAY_INTERSECTION . 1)
    (:THREE_WAY_ROUNDABOUT . 2)
    (:NORTH . 0)
    (:SOUTH . 1)
    (:EAST . 2)
    (:WEST . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CDMSection)))
    "Constants for message type 'CDMSection"
  '((:THREE_WAY_INTERSECTION . 0)
    (:FOUR_WAY_INTERSECTION . 1)
    (:THREE_WAY_ROUNDABOUT . 2)
    (:NORTH . 0)
    (:SOUTH . 1)
    (:EAST . 2)
    (:WEST . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CDMSection>) ostream)
  "Serializes a message object of type '<CDMSection>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'orientation)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'connection_zones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'connection_zones))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CDMSection>) istream)
  "Deserializes a message object of type '<CDMSection>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'orientation) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'connection_zones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'connection_zones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mapdata-msg:CDMConnectionZone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CDMSection>)))
  "Returns string type for a message object of type '<CDMSection>"
  "mapdata/CDMSection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CDMSection)))
  "Returns string type for a message object of type 'CDMSection"
  "mapdata/CDMSection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CDMSection>)))
  "Returns md5sum for a message object of type '<CDMSection>"
  "84a37099866d8390b55fd98c37e4e716")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CDMSection)))
  "Returns md5sum for a message object of type 'CDMSection"
  "84a37099866d8390b55fd98c37e4e716")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CDMSection>)))
  "Returns full string definition for message of type '<CDMSection>"
  (cl:format cl:nil "# Type enumeration~%uint8 THREE_WAY_INTERSECTION=0~%uint8 FOUR_WAY_INTERSECTION=1~%uint8 THREE_WAY_ROUNDABOUT=2~%~%# Orientation enumeration~%uint8 NORTH=0~%uint8 SOUTH=1~%uint8 EAST=2~%uint8 WEST=3~%~%~%int32 id~%Position pos~%int32 type~%int32 width~%int32 height~%int32 orientation~%CDMConnectionZone[] connection_zones~%~%================================================================================~%MSG: mapdata/Position~%int32 x~%int32 y~%~%================================================================================~%MSG: mapdata/CDMConnectionZone~%Position pos~%uint32 width~%uint32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CDMSection)))
  "Returns full string definition for message of type 'CDMSection"
  (cl:format cl:nil "# Type enumeration~%uint8 THREE_WAY_INTERSECTION=0~%uint8 FOUR_WAY_INTERSECTION=1~%uint8 THREE_WAY_ROUNDABOUT=2~%~%# Orientation enumeration~%uint8 NORTH=0~%uint8 SOUTH=1~%uint8 EAST=2~%uint8 WEST=3~%~%~%int32 id~%Position pos~%int32 type~%int32 width~%int32 height~%int32 orientation~%CDMConnectionZone[] connection_zones~%~%================================================================================~%MSG: mapdata/Position~%int32 x~%int32 y~%~%================================================================================~%MSG: mapdata/CDMConnectionZone~%Position pos~%uint32 width~%uint32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CDMSection>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'connection_zones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CDMSection>))
  "Converts a ROS message object to a list"
  (cl:list 'CDMSection
    (cl:cons ':id (id msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':type (type msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':connection_zones (connection_zones msg))
))
