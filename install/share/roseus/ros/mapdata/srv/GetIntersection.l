;; Auto-generated. Do not edit!


(when (boundp 'mapdata::GetIntersection)
  (if (not (find-package "MAPDATA"))
    (make-package "MAPDATA"))
  (shadow 'GetIntersection (find-package "MAPDATA")))
(unless (find-package "MAPDATA::GETINTERSECTION")
  (make-package "MAPDATA::GETINTERSECTION"))
(unless (find-package "MAPDATA::GETINTERSECTIONREQUEST")
  (make-package "MAPDATA::GETINTERSECTIONREQUEST"))
(unless (find-package "MAPDATA::GETINTERSECTIONRESPONSE")
  (make-package "MAPDATA::GETINTERSECTIONRESPONSE"))

(in-package "ROS")





(defclass mapdata::GetIntersectionRequest
  :super ros::object
  :slots ())

(defmethod mapdata::GetIntersectionRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass mapdata::GetIntersectionResponse
  :super ros::object
  :slots (_north _west _south _east ))

(defmethod mapdata::GetIntersectionResponse
  (:init
   (&key
    ((:north __north) (instance mapdata::RoadSection :init))
    ((:west __west) (instance mapdata::RoadSection :init))
    ((:south __south) (instance mapdata::RoadSection :init))
    ((:east __east) (instance mapdata::RoadSection :init))
    )
   (send-super :init)
   (setq _north __north)
   (setq _west __west)
   (setq _south __south)
   (setq _east __east)
   self)
  (:north
   (&rest __north)
   (if (keywordp (car __north))
       (send* _north __north)
     (progn
       (if __north (setq _north (car __north)))
       _north)))
  (:west
   (&rest __west)
   (if (keywordp (car __west))
       (send* _west __west)
     (progn
       (if __west (setq _west (car __west)))
       _west)))
  (:south
   (&rest __south)
   (if (keywordp (car __south))
       (send* _south __south)
     (progn
       (if __south (setq _south (car __south)))
       _south)))
  (:east
   (&rest __east)
   (if (keywordp (car __east))
       (send* _east __east)
     (progn
       (if __east (setq _east (car __east)))
       _east)))
  (:serialization-length
   ()
   (+
    ;; mapdata/RoadSection _north
    (send _north :serialization-length)
    ;; mapdata/RoadSection _west
    (send _west :serialization-length)
    ;; mapdata/RoadSection _south
    (send _south :serialization-length)
    ;; mapdata/RoadSection _east
    (send _east :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; mapdata/RoadSection _north
       (send _north :serialize s)
     ;; mapdata/RoadSection _west
       (send _west :serialize s)
     ;; mapdata/RoadSection _south
       (send _south :serialize s)
     ;; mapdata/RoadSection _east
       (send _east :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; mapdata/RoadSection _north
     (send _north :deserialize buf ptr-) (incf ptr- (send _north :serialization-length))
   ;; mapdata/RoadSection _west
     (send _west :deserialize buf ptr-) (incf ptr- (send _west :serialization-length))
   ;; mapdata/RoadSection _south
     (send _south :deserialize buf ptr-) (incf ptr- (send _south :serialization-length))
   ;; mapdata/RoadSection _east
     (send _east :deserialize buf ptr-) (incf ptr- (send _east :serialization-length))
   ;;
   self)
  )

(defclass mapdata::GetIntersection
  :super ros::object
  :slots ())

(setf (get mapdata::GetIntersection :md5sum-) "05e0de0c4f7d86e84748f13d311cd03e")
(setf (get mapdata::GetIntersection :datatype-) "mapdata/GetIntersection")
(setf (get mapdata::GetIntersection :request) mapdata::GetIntersectionRequest)
(setf (get mapdata::GetIntersection :response) mapdata::GetIntersectionResponse)

(defmethod mapdata::GetIntersectionRequest
  (:response () (instance mapdata::GetIntersectionResponse :init)))

(setf (get mapdata::GetIntersectionRequest :md5sum-) "05e0de0c4f7d86e84748f13d311cd03e")
(setf (get mapdata::GetIntersectionRequest :datatype-) "mapdata/GetIntersectionRequest")
(setf (get mapdata::GetIntersectionRequest :definition-)
      "---
# For our purposes a 4-way intersection is enough
RoadSection north
RoadSection west
RoadSection south
RoadSection east

================================================================================
MSG: mapdata/RoadSection
# If you're standing on the road looking towards the intersection, the left
# position is the leftmost edge of the rectangle closest to the intersection.
#    |       |
#    |       |
#  right    left
# ---+       +----
#
Position left
Position right

# How far the road section extends from the intersection
int32 length

# How far from the intersection the cars should stop
int32 stopline_offset

# Enumeration (just constants) of priority signs
uint8 PRIORITY_ROAD=0
uint8 GIVE_WAY=1
uint8 STOP_SIGN=2
uint8 TRAFFIC_LIGHT=3
uint8 BOOKING=4

uint8 priority_sign

# A bit redundant but nice for pretty printing
string name

================================================================================
MSG: mapdata/Position
int32 x
int32 y
")

(setf (get mapdata::GetIntersectionResponse :md5sum-) "05e0de0c4f7d86e84748f13d311cd03e")
(setf (get mapdata::GetIntersectionResponse :datatype-) "mapdata/GetIntersectionResponse")
(setf (get mapdata::GetIntersectionResponse :definition-)
      "---
# For our purposes a 4-way intersection is enough
RoadSection north
RoadSection west
RoadSection south
RoadSection east

================================================================================
MSG: mapdata/RoadSection
# If you're standing on the road looking towards the intersection, the left
# position is the leftmost edge of the rectangle closest to the intersection.
#    |       |
#    |       |
#  right    left
# ---+       +----
#
Position left
Position right

# How far the road section extends from the intersection
int32 length

# How far from the intersection the cars should stop
int32 stopline_offset

# Enumeration (just constants) of priority signs
uint8 PRIORITY_ROAD=0
uint8 GIVE_WAY=1
uint8 STOP_SIGN=2
uint8 TRAFFIC_LIGHT=3
uint8 BOOKING=4

uint8 priority_sign

# A bit redundant but nice for pretty printing
string name

================================================================================
MSG: mapdata/Position
int32 x
int32 y
")



(provide :mapdata/GetIntersection "05e0de0c4f7d86e84748f13d311cd03e")


