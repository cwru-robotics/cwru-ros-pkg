(defpackage navfn-srv
  (:use cl
        roslisp-msg-protocol)
  (:export
   "MAKENAVPLAN"
   "<MAKENAVPLAN-REQUEST>"
   "<MAKENAVPLAN-RESPONSE>"
   "SETCOSTMAP"
   "<SETCOSTMAP-REQUEST>"
   "<SETCOSTMAP-RESPONSE>"
  ))

