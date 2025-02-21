;; Auto-generated. Do not edit!


(when (boundp 'roswifibot::IR)
  (if (not (find-package "ROSWIFIBOT"))
    (make-package "ROSWIFIBOT"))
  (shadow 'IR (find-package "ROSWIFIBOT")))
(unless (find-package "ROSWIFIBOT::IR")
  (make-package "ROSWIFIBOT::IR"))

(in-package "ROS")
;;//! \htmlinclude IR.msg.html


(defclass roswifibot::IR
  :super ros::object
  :slots (_IR_front_left _IR_back_left _IR_front_right _IR_back_right ))

(defmethod roswifibot::IR
  (:init
   (&key
    ((:IR_front_left __IR_front_left) 0.0)
    ((:IR_back_left __IR_back_left) 0.0)
    ((:IR_front_right __IR_front_right) 0.0)
    ((:IR_back_right __IR_back_right) 0.0)
    )
   (send-super :init)
   (setq _IR_front_left (float __IR_front_left))
   (setq _IR_back_left (float __IR_back_left))
   (setq _IR_front_right (float __IR_front_right))
   (setq _IR_back_right (float __IR_back_right))
   self)
  (:IR_front_left
   (&optional __IR_front_left)
   (if __IR_front_left (setq _IR_front_left __IR_front_left)) _IR_front_left)
  (:IR_back_left
   (&optional __IR_back_left)
   (if __IR_back_left (setq _IR_back_left __IR_back_left)) _IR_back_left)
  (:IR_front_right
   (&optional __IR_front_right)
   (if __IR_front_right (setq _IR_front_right __IR_front_right)) _IR_front_right)
  (:IR_back_right
   (&optional __IR_back_right)
   (if __IR_back_right (setq _IR_back_right __IR_back_right)) _IR_back_right)
  (:serialization-length
   ()
   (+
    ;; float64 _IR_front_left
    8
    ;; float64 _IR_back_left
    8
    ;; float64 _IR_front_right
    8
    ;; float64 _IR_back_right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _IR_front_left
       (sys::poke _IR_front_left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR_back_left
       (sys::poke _IR_back_left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR_front_right
       (sys::poke _IR_front_right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _IR_back_right
       (sys::poke _IR_back_right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _IR_front_left
     (setq _IR_front_left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR_back_left
     (setq _IR_back_left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR_front_right
     (setq _IR_front_right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _IR_back_right
     (setq _IR_back_right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get roswifibot::IR :md5sum-) "08989c603acc510242caf5149106a2a8")
(setf (get roswifibot::IR :datatype-) "roswifibot/IR")
(setf (get roswifibot::IR :definition-)
      "float64 IR_front_left
float64 IR_back_left
float64 IR_front_right
float64 IR_back_right
")



(provide :roswifibot/IR "08989c603acc510242caf5149106a2a8")


