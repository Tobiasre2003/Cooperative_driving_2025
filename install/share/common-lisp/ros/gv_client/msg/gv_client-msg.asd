
(cl:in-package :asdf)

(defsystem "gv_client-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GulliViewPosition" :depends-on ("_package_GulliViewPosition"))
    (:file "_package_GulliViewPosition" :depends-on ("_package"))
    (:file "LaptopSpeed" :depends-on ("_package_LaptopSpeed"))
    (:file "_package_LaptopSpeed" :depends-on ("_package"))
  ))