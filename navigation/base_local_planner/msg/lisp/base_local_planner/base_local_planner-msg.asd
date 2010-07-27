
(in-package :asdf)

(defsystem "base_local_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "CmdVel_viz" :depends-on ("_package"))
    (:file "_package_CmdVel_viz" :depends-on ("_package"))
    (:file "Position2DInt" :depends-on ("_package"))
    (:file "_package_Position2DInt" :depends-on ("_package"))
    ))
