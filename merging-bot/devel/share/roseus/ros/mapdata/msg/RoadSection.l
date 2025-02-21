;; Auto-generated. Do not edit!


(when (boundp 'mapdata::RoadSection)
  (if (not (find-package "MAPDATA"))
    (make-package "MAPDATA"))
  (shadow 'RoadSection (find-package "MAPDATA")))
(unless (find-package "MAPDATA::ROADSECTION")
  (make-package "MAPDATA::ROADSECTION"))

(in-package "ROS")
;;//! \htmlinclude RoadSection.msg.html


(intern "*PRIORITY_ROAD*" (find-package "MAPDATA::ROADSECTION"))
(shadow '*PRIORITY_ROAD* (find-package "MAPDATA::ROADSECTION"))
(defconstant mapdata::RoadSection::*PRIORITY_ROAD* 0)
(intern "*GIVE_WAY*" (find-package "MAPDATA::ROADSECTION"))
(shadow '*GIVE_WAY* (find-package "MAPDATA::ROADSECTION"))
(defconstant mapdata::RoadSection::*GIVE_WAY* 1)
(intern "*STOP_SIGN*" (find-package "MAPDATA::ROADSECTION"))
(shadow '*STOP_SIGN* (find-package "MAPDATA::ROADSECTION"))
(defconstant mapdata::RoadSection::*STOP_SIGN* 2)
(intern "*TRAFFIC_LIGHT*" (find-package "MAPDATA::ROADSECTION"))
(shadow '*TRAFFIC_LIGHT* (find-package "MAPDATA::ROADSECTION"))
(defconstant mapdata::RoadSection::*TRAFFIC_LIGHT* 3)
(intern "*BOOKING*" (find-package "MAPDATA::ROADSECTION"))
(shadow '*BOOKING* (find-package "MAPDATA::ROADSECTION"))
(defconstant mapdata::RoadSection::*BOOKING* 4)

(defun mapdata::RoadSection-to-symbol (const)
  (cond
        ((= const 0) 'mapdata::RoadSection::*PRIORITY_ROAD*)
        ((= const 1) 'mapdata::RoadSection::*GIVE_WAY*)
        ((= const 2) 'mapdata::RoadSection::*STOP_SIGN*)
        ((= const 3) 'mapdata::RoadSection::*TRAFFIC_LIGHT*)
        ((= const 4) 'mapdata::RoadSection::*BOOKING*)
        (t nil)))

(defclass mapdata::RoadSection
  :super ros::object
  :slots (_left _right _length _stopline_offset _priority_sign _name ))

(defmethod mapdata::RoadSection
  (:init
   (&key
    ((:left __left) (instance mapdata::Position :init))
    ((:right __right) (instance mapdata::Position :init))
    ((:length __length) 0)
    ((:stopline_offset __stopline_offset) 0)
    ((:priority_sign __priority_sign) 0)
    ((:name __name) "")
    )
   (send-super :init)
   (setq _left __left)
   (setq _right __right)
   (setq _length (round __length))
   (setq _stopline_offset (round __stopline_offset))
   (setq _priority_sign (round __priority_sign))
   (setq _name (string __name))
   self)
  (:left
   (&rest __left)
   (if (keywordp (car __left))
       (send* _left __left)
     (progn
       (if __left (setq _left (car __left)))
       _left)))
  (:right
   (&rest __right)
   (if (keywordp (car __right))
       (send* _right __right)
     (progn
       (if __right (setq _right (car __right)))
       _right)))
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:stopline_offset
   (&optional __stopline_offset)
   (if __stopline_offset (setq _stopline_offset __stopline_offset)) _stopline_offset)
  (:priority_sign
   (&optional __priority_sign)
   (if __priority_sign (setq _priority_sign __priority_sign)) _priority_sign)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; mapdata/Position _left
    (send _left :serialization-length)
    ;; mapdata/Position _right
    (send _right :serialization-length)
    ;; int32 _length
    4
    ;; int32 _stopline_offset
    4
    ;; uint8 _priority_sign
    1
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; mapdata/Position _left
       (send _left :serialize s)
     ;; mapdata/Position _right
       (send _right :serialize s)
     ;; int32 _length
       (write-long _length s)
     ;; int32 _stopline_offset
       (write-long _stopline_offset s)
     ;; uint8 _priority_sign
       (write-byte _priority_sign s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; mapdata/Position _left
     (send _left :deserialize buf ptr-) (incf ptr- (send _left :serialization-length))
   ;; mapdata/Position _right
     (send _right :deserialize buf ptr-) (incf ptr- (send _right :serialization-length))
   ;; int32 _length
     (setq _length (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _stopline_offset
     (setq _stopline_offset (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _priority_sign
     (setq _priority_sign (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get mapdata::RoadSection :md5sum-) "e564107726b89210b4492752f37581d9")
(setf (get mapdata::RoadSection :datatype-) "mapdata/RoadSection")
(setf (get mapdata::RoadSection :definition-)
      "# If you're standing on the road looking towards the intersection, the left
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



(provide :mapdata/RoadSection "e564107726b89210b4492752f37581d9")


