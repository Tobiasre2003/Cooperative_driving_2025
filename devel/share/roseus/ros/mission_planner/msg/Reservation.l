;; Auto-generated. Do not edit!


(when (boundp 'mission_planner::Reservation)
  (if (not (find-package "MISSION_PLANNER"))
    (make-package "MISSION_PLANNER"))
  (shadow 'Reservation (find-package "MISSION_PLANNER")))
(unless (find-package "MISSION_PLANNER::RESERVATION")
  (make-package "MISSION_PLANNER::RESERVATION"))

(in-package "ROS")
;;//! \htmlinclude Reservation.msg.html


(defclass mission_planner::Reservation
  :super ros::object
  :slots (_start _end ))

(defmethod mission_planner::Reservation
  (:init
   (&key
    ((:start __start) "")
    ((:end __end) "")
    )
   (send-super :init)
   (setq _start (string __start))
   (setq _end (string __end))
   self)
  (:start
   (&optional __start)
   (if __start (setq _start __start)) _start)
  (:end
   (&optional __end)
   (if __end (setq _end __end)) _end)
  (:serialization-length
   ()
   (+
    ;; string _start
    4 (length _start)
    ;; string _end
    4 (length _end)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _start
       (write-long (length _start) s) (princ _start s)
     ;; string _end
       (write-long (length _end) s) (princ _end s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _start
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _start (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _end
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _end (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get mission_planner::Reservation :md5sum-) "fd9b411e8e594457e56809ca789fc69e")
(setf (get mission_planner::Reservation :datatype-) "mission_planner/Reservation")
(setf (get mission_planner::Reservation :definition-)
      "string start
string end
")



(provide :mission_planner/Reservation "fd9b411e8e594457e56809ca789fc69e")


