;; Auto-generated. Do not edit!


(when (boundp 'mapdata::CDMSection)
  (if (not (find-package "MAPDATA"))
    (make-package "MAPDATA"))
  (shadow 'CDMSection (find-package "MAPDATA")))
(unless (find-package "MAPDATA::CDMSECTION")
  (make-package "MAPDATA::CDMSECTION"))

(in-package "ROS")
;;//! \htmlinclude CDMSection.msg.html


(intern "*THREE_WAY_INTERSECTION*" (find-package "MAPDATA::CDMSECTION"))
(shadow '*THREE_WAY_INTERSECTION* (find-package "MAPDATA::CDMSECTION"))
(defconstant mapdata::CDMSection::*THREE_WAY_INTERSECTION* 0)
(intern "*FOUR_WAY_INTERSECTION*" (find-package "MAPDATA::CDMSECTION"))
(shadow '*FOUR_WAY_INTERSECTION* (find-package "MAPDATA::CDMSECTION"))
(defconstant mapdata::CDMSection::*FOUR_WAY_INTERSECTION* 1)
(intern "*THREE_WAY_ROUNDABOUT*" (find-package "MAPDATA::CDMSECTION"))
(shadow '*THREE_WAY_ROUNDABOUT* (find-package "MAPDATA::CDMSECTION"))
(defconstant mapdata::CDMSection::*THREE_WAY_ROUNDABOUT* 2)
(intern "*NORTH*" (find-package "MAPDATA::CDMSECTION"))
(shadow '*NORTH* (find-package "MAPDATA::CDMSECTION"))
(defconstant mapdata::CDMSection::*NORTH* 0)
(intern "*SOUTH*" (find-package "MAPDATA::CDMSECTION"))
(shadow '*SOUTH* (find-package "MAPDATA::CDMSECTION"))
(defconstant mapdata::CDMSection::*SOUTH* 1)
(intern "*EAST*" (find-package "MAPDATA::CDMSECTION"))
(shadow '*EAST* (find-package "MAPDATA::CDMSECTION"))
(defconstant mapdata::CDMSection::*EAST* 2)
(intern "*WEST*" (find-package "MAPDATA::CDMSECTION"))
(shadow '*WEST* (find-package "MAPDATA::CDMSECTION"))
(defconstant mapdata::CDMSection::*WEST* 3)

(defun mapdata::CDMSection-to-symbol (const)
  (cond
        ((= const 0) 'mapdata::CDMSection::*THREE_WAY_INTERSECTION*)
        ((= const 1) 'mapdata::CDMSection::*FOUR_WAY_INTERSECTION*)
        ((= const 2) 'mapdata::CDMSection::*THREE_WAY_ROUNDABOUT*)
        ((= const 0) 'mapdata::CDMSection::*NORTH*)
        ((= const 1) 'mapdata::CDMSection::*SOUTH*)
        ((= const 2) 'mapdata::CDMSection::*EAST*)
        ((= const 3) 'mapdata::CDMSection::*WEST*)
        (t nil)))

(defclass mapdata::CDMSection
  :super ros::object
  :slots (_id _pos _type _width _height _orientation _connection_zones ))

(defmethod mapdata::CDMSection
  (:init
   (&key
    ((:id __id) 0)
    ((:pos __pos) (instance mapdata::Position :init))
    ((:type __type) 0)
    ((:width __width) 0)
    ((:height __height) 0)
    ((:orientation __orientation) 0)
    ((:connection_zones __connection_zones) ())
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pos __pos)
   (setq _type (round __type))
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _orientation (round __orientation))
   (setq _connection_zones __connection_zones)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:orientation
   (&optional __orientation)
   (if __orientation (setq _orientation __orientation)) _orientation)
  (:connection_zones
   (&rest __connection_zones)
   (if (keywordp (car __connection_zones))
       (send* _connection_zones __connection_zones)
     (progn
       (if __connection_zones (setq _connection_zones (car __connection_zones)))
       _connection_zones)))
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; mapdata/Position _pos
    (send _pos :serialization-length)
    ;; int32 _type
    4
    ;; int32 _width
    4
    ;; int32 _height
    4
    ;; int32 _orientation
    4
    ;; mapdata/CDMConnectionZone[] _connection_zones
    (apply #'+ (send-all _connection_zones :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; mapdata/Position _pos
       (send _pos :serialize s)
     ;; int32 _type
       (write-long _type s)
     ;; int32 _width
       (write-long _width s)
     ;; int32 _height
       (write-long _height s)
     ;; int32 _orientation
       (write-long _orientation s)
     ;; mapdata/CDMConnectionZone[] _connection_zones
     (write-long (length _connection_zones) s)
     (dolist (elem _connection_zones)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; mapdata/Position _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _orientation
     (setq _orientation (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; mapdata/CDMConnectionZone[] _connection_zones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _connection_zones (let (r) (dotimes (i n) (push (instance mapdata::CDMConnectionZone :init) r)) r))
     (dolist (elem- _connection_zones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get mapdata::CDMSection :md5sum-) "84a37099866d8390b55fd98c37e4e716")
(setf (get mapdata::CDMSection :datatype-) "mapdata/CDMSection")
(setf (get mapdata::CDMSection :definition-)
      "# Type enumeration
uint8 THREE_WAY_INTERSECTION=0
uint8 FOUR_WAY_INTERSECTION=1
uint8 THREE_WAY_ROUNDABOUT=2

# Orientation enumeration
uint8 NORTH=0
uint8 SOUTH=1
uint8 EAST=2
uint8 WEST=3


int32 id
Position pos
int32 type
int32 width
int32 height
int32 orientation
CDMConnectionZone[] connection_zones

================================================================================
MSG: mapdata/Position
int32 x
int32 y

================================================================================
MSG: mapdata/CDMConnectionZone
Position pos
uint32 width
uint32 height

")



(provide :mapdata/CDMSection "84a37099866d8390b55fd98c37e4e716")


