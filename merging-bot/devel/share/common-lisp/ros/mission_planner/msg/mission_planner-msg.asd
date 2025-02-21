
(cl:in-package :asdf)

(defsystem "mission_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Reservation" :depends-on ("_package_Reservation"))
    (:file "_package_Reservation" :depends-on ("_package"))
  ))