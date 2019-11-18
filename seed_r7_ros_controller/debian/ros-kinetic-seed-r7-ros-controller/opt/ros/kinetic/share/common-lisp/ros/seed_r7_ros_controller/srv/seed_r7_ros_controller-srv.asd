
(cl:in-package :asdf)

(defsystem "seed_r7_ros_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "HandControl" :depends-on ("_package_HandControl"))
    (:file "_package_HandControl" :depends-on ("_package"))
  ))