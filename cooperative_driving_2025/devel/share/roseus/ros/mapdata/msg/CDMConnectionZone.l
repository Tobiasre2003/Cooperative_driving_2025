;; Auto-generated. Do not edit!


(when (boundp 'mapdata::CDMConnectionZone)
  (if (not (find-package "MAPDATA"))
    (make-package "MAPDATA"))
  (shadow 'CDMConnectionZone (find-package "MAPDATA")))
(unless (find-package "MAPDATA::CDMCONNECTIONZONE")
  (make-package "MAPDATA::CDMCONNECTIONZONE"))

(in-package "ROS")
;;//! \htmlinclude CDMConnectionZone.msg.html


(defclass mapdata::CDMConnectionZone
  :super ros::object
  :slots (_pos _width _height ))

(defmethod mapdata::CDMConnectionZone
  (:init
   (&key
    ((:pos __pos) (instance mapdata::Position :init))
    ((:width __width) 0)
    ((:height __height) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _width (round __width))
   (setq _height (round __height))
   self)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; mapdata/Position _pos
    (send _pos :serialization-length)
    ;; uint32 _width
    4
    ;; uint32 _height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; mapdata/Position _pos
       (send _pos :serialize s)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _height
       (write-long _height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; mapdata/Position _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mapdata::CDMConnectionZone :md5sum-) "49ee6d673b89f273a2281d8ddb4755f9")
(setf (get mapdata::CDMConnectionZone :datatype-) "mapdata/CDMConnectionZone")
(setf (get mapdata::CDMConnectionZone :definition-)
      "Position pos
uint32 width
uint32 height

================================================================================
MSG: mapdata/Position
int32 x
int32 y

")



(provide :mapdata/CDMConnectionZone "49ee6d673b89f273a2281d8ddb4755f9")


