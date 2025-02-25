; Auto-generated. Do not edit!


(cl:in-package mapdata-srv)


;//! \htmlinclude GetIntersection-request.msg.html

(cl:defclass <GetIntersection-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetIntersection-request (<GetIntersection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntersection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntersection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapdata-srv:<GetIntersection-request> is deprecated: use mapdata-srv:GetIntersection-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntersection-request>) ostream)
  "Serializes a message object of type '<GetIntersection-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntersection-request>) istream)
  "Deserializes a message object of type '<GetIntersection-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntersection-request>)))
  "Returns string type for a service object of type '<GetIntersection-request>"
  "mapdata/GetIntersectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntersection-request)))
  "Returns string type for a service object of type 'GetIntersection-request"
  "mapdata/GetIntersectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntersection-request>)))
  "Returns md5sum for a message object of type '<GetIntersection-request>"
  "05e0de0c4f7d86e84748f13d311cd03e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntersection-request)))
  "Returns md5sum for a message object of type 'GetIntersection-request"
  "05e0de0c4f7d86e84748f13d311cd03e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntersection-request>)))
  "Returns full string definition for message of type '<GetIntersection-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntersection-request)))
  "Returns full string definition for message of type 'GetIntersection-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntersection-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntersection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntersection-request
))
;//! \htmlinclude GetIntersection-response.msg.html

(cl:defclass <GetIntersection-response> (roslisp-msg-protocol:ros-message)
  ((north
    :reader north
    :initarg :north
    :type mapdata-msg:RoadSection
    :initform (cl:make-instance 'mapdata-msg:RoadSection))
   (west
    :reader west
    :initarg :west
    :type mapdata-msg:RoadSection
    :initform (cl:make-instance 'mapdata-msg:RoadSection))
   (south
    :reader south
    :initarg :south
    :type mapdata-msg:RoadSection
    :initform (cl:make-instance 'mapdata-msg:RoadSection))
   (east
    :reader east
    :initarg :east
    :type mapdata-msg:RoadSection
    :initform (cl:make-instance 'mapdata-msg:RoadSection)))
)

(cl:defclass GetIntersection-response (<GetIntersection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntersection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntersection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapdata-srv:<GetIntersection-response> is deprecated: use mapdata-srv:GetIntersection-response instead.")))

(cl:ensure-generic-function 'north-val :lambda-list '(m))
(cl:defmethod north-val ((m <GetIntersection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-srv:north-val is deprecated.  Use mapdata-srv:north instead.")
  (north m))

(cl:ensure-generic-function 'west-val :lambda-list '(m))
(cl:defmethod west-val ((m <GetIntersection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-srv:west-val is deprecated.  Use mapdata-srv:west instead.")
  (west m))

(cl:ensure-generic-function 'south-val :lambda-list '(m))
(cl:defmethod south-val ((m <GetIntersection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-srv:south-val is deprecated.  Use mapdata-srv:south instead.")
  (south m))

(cl:ensure-generic-function 'east-val :lambda-list '(m))
(cl:defmethod east-val ((m <GetIntersection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapdata-srv:east-val is deprecated.  Use mapdata-srv:east instead.")
  (east m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntersection-response>) ostream)
  "Serializes a message object of type '<GetIntersection-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'north) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'west) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'south) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'east) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntersection-response>) istream)
  "Deserializes a message object of type '<GetIntersection-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'north) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'west) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'south) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'east) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntersection-response>)))
  "Returns string type for a service object of type '<GetIntersection-response>"
  "mapdata/GetIntersectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntersection-response)))
  "Returns string type for a service object of type 'GetIntersection-response"
  "mapdata/GetIntersectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntersection-response>)))
  "Returns md5sum for a message object of type '<GetIntersection-response>"
  "05e0de0c4f7d86e84748f13d311cd03e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntersection-response)))
  "Returns md5sum for a message object of type 'GetIntersection-response"
  "05e0de0c4f7d86e84748f13d311cd03e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntersection-response>)))
  "Returns full string definition for message of type '<GetIntersection-response>"
  (cl:format cl:nil "# For our purposes a 4-way intersection is enough~%RoadSection north~%RoadSection west~%RoadSection south~%RoadSection east~%~%================================================================================~%MSG: mapdata/RoadSection~%# If you're standing on the road looking towards the intersection, the left~%# position is the leftmost edge of the rectangle closest to the intersection.~%#    |       |~%#    |       |~%#  right    left~%# ---+       +----~%#~%Position left~%Position right~%~%# How far the road section extends from the intersection~%int32 length~%~%# How far from the intersection the cars should stop~%int32 stopline_offset~%~%# Enumeration (just constants) of priority signs~%uint8 PRIORITY_ROAD=0~%uint8 GIVE_WAY=1~%uint8 STOP_SIGN=2~%uint8 TRAFFIC_LIGHT=3~%uint8 BOOKING=4~%~%uint8 priority_sign~%~%# A bit redundant but nice for pretty printing~%string name~%~%================================================================================~%MSG: mapdata/Position~%int32 x~%int32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntersection-response)))
  "Returns full string definition for message of type 'GetIntersection-response"
  (cl:format cl:nil "# For our purposes a 4-way intersection is enough~%RoadSection north~%RoadSection west~%RoadSection south~%RoadSection east~%~%================================================================================~%MSG: mapdata/RoadSection~%# If you're standing on the road looking towards the intersection, the left~%# position is the leftmost edge of the rectangle closest to the intersection.~%#    |       |~%#    |       |~%#  right    left~%# ---+       +----~%#~%Position left~%Position right~%~%# How far the road section extends from the intersection~%int32 length~%~%# How far from the intersection the cars should stop~%int32 stopline_offset~%~%# Enumeration (just constants) of priority signs~%uint8 PRIORITY_ROAD=0~%uint8 GIVE_WAY=1~%uint8 STOP_SIGN=2~%uint8 TRAFFIC_LIGHT=3~%uint8 BOOKING=4~%~%uint8 priority_sign~%~%# A bit redundant but nice for pretty printing~%string name~%~%================================================================================~%MSG: mapdata/Position~%int32 x~%int32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntersection-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'north))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'west))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'south))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'east))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntersection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntersection-response
    (cl:cons ':north (north msg))
    (cl:cons ':west (west msg))
    (cl:cons ':south (south msg))
    (cl:cons ':east (east msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIntersection)))
  'GetIntersection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIntersection)))
  'GetIntersection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntersection)))
  "Returns string type for a service object of type '<GetIntersection>"
  "mapdata/GetIntersection")