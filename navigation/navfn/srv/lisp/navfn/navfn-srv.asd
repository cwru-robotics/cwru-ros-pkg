
(in-package :asdf)

(defsystem "navfn-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg :roslib-msg)
  :components ((:file "_package")
    (:file "MakeNavPlan" :depends-on ("_package"))
    (:file "_package_MakeNavPlan" :depends-on ("_package"))
    (:file "SetCostmap" :depends-on ("_package"))
    (:file "_package_SetCostmap" :depends-on ("_package"))
    ))
