;; Auto-generated. Do not edit!


(when (boundp 'gv_client::LaptopSpeed)
  (if (not (find-package "GV_CLIENT"))
    (make-package "GV_CLIENT"))
  (shadow 'LaptopSpeed (find-package "GV_CLIENT")))
(unless (find-package "GV_CLIENT::LAPTOPSPEED")
  (make-package "GV_CLIENT::LAPTOPSPEED"))

(in-package "ROS")
;;//! \htmlinclude LaptopSpeed.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gv_client::LaptopSpeed
  :super ros::object
  :slots (_header _tag_id _speed ))

(defmethod gv_client::LaptopSpeed
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:tag_id __tag_id) 0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _tag_id (round __tag_id))
   (setq _speed (float __speed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:tag_id
   (&optional __tag_id)
   (if __tag_id (setq _tag_id __tag_id)) _tag_id)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _tag_id
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _tag_id
       (write-long _tag_id s)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _tag_id
     (setq _tag_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get gv_client::LaptopSpeed :md5sum-) "e0bd5e9fcfaeea2bcb9357113ff221f2")
(setf (get gv_client::LaptopSpeed :datatype-) "gv_client/LaptopSpeed")
(setf (get gv_client::LaptopSpeed :definition-)
      "Header header
int32 tag_id
float32 speed

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



(provide :gv_client/LaptopSpeed "e0bd5e9fcfaeea2bcb9357113ff221f2")


