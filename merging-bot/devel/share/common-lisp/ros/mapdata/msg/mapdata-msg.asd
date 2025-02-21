
(cl:in-package :asdf)

(defsystem "mapdata-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CDMConnectionZone" :depends-on ("_package_CDMConnectionZone"))
    (:file "_package_CDMConnectionZone" :depends-on ("_package"))
    (:file "CDMSection" :depends-on ("_package_CDMSection"))
    (:file "_package_CDMSection" :depends-on ("_package"))
    (:file "Position" :depends-on ("_package_Position"))
    (:file "_package_Position" :depends-on ("_package"))
    (:file "RoadSection" :depends-on ("_package_RoadSection"))
    (:file "_package_RoadSection" :depends-on ("_package"))
  ))