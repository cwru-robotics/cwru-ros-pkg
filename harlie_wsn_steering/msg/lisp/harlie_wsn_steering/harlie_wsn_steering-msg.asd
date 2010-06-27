
(in-package :asdf)

(defsystem "harlie_wsn_steering-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "DesiredState" :depends-on ("_package"))
    (:file "_package_DesiredState" :depends-on ("_package"))
    (:file "PathSegment" :depends-on ("_package"))
    (:file "_package_PathSegment" :depends-on ("_package"))
    ))
