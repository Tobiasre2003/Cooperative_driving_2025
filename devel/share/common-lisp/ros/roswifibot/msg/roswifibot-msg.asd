
(cl:in-package :asdf)

(defsystem "roswifibot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "IR" :depends-on ("_package_IR"))
    (:file "_package_IR" :depends-on ("_package"))
    (:file "Status" :depends-on ("_package_Status"))
    (:file "_package_Status" :depends-on ("_package"))
  ))