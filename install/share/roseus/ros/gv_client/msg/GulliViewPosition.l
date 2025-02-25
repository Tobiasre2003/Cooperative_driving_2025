;; Auto-generated. Do not edit!


(when (boundp 'gv_client::GulliViewPosition)
  (if (not (find-package "GV_CLIENT"))
    (make-package "GV_CLIENT"))
  (shadow 'GulliViewPosition (find-package "GV_CLIENT")))
(unless (find-package "GV_CLIENT::GULLIVIEWPOSITION")
  (make-package "GV_CLIENT::GULLIVIEWPOSITION"))

(in-package "ROS")
;;//! \htmlinclude GulliViewPosition.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gv_client::GulliViewPosition
  :super ros::object
  :slots (_header _x _y _theta _speed _tagId _cameraId ))

(defmethod gv_client::GulliViewPosition
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0)
    ((:y __y) 0)
    ((:theta __theta) 0.0)
    ((:speed __speed) 0.0)
    ((:tagId __tagId) 0)
    ((:cameraId __cameraId) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _theta (float __theta))
   (setq _speed (float __speed))
   (setq _tagId (round __tagId))
   (setq _cameraId (round __cameraId))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:tagId
   (&optional __tagId)
   (if __tagId (setq _tagId __tagId)) _tagId)
  (:cameraId
   (&optional __cameraId)
   (if __cameraId (setq _cameraId __cameraId)) _cameraId)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; float32 _theta
    4
    ;; float32 _speed
    4
    ;; int32 _tagId
    4
    ;; int32 _cameraId
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _tagId
       (write-long _tagId s)
     ;; int32 _cameraId
       (write-long _cameraId s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _tagId
     (setq _tagId (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _cameraId
     (setq _cameraId (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get gv_client::GulliViewPosition :md5sum-) "422884b87830ecb9e6166854972efe25")
(setf (get gv_client::GulliViewPosition :datatype-) "gv_client/GulliViewPosition")
(setf (get gv_client::GulliViewPosition :definition-)
      "Header header
int32 x
int32 y
float32 theta
float32 speed
int32 tagId
int32 cameraId

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :gv_client/GulliViewPosition "422884b87830ecb9e6166854972efe25")


