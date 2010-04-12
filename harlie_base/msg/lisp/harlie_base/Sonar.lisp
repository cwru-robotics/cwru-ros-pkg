; Auto-generated. Do not edit!


(in-package harlie_base-msg)


;//! \htmlinclude Sonar.msg.html

(defclass <Sonar> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (dist
    :reader dist-val
    :initarg :dist
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Sonar>) ostream)
  "Serializes a message object of type '<Sonar>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'dist))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <Sonar>) istream)
  "Deserializes a message object of type '<Sonar>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'dist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Sonar>)))
  "Returns string type for a message object of type '<Sonar>"
  "harlie_base/Sonar")
(defmethod md5sum ((type (eql '<Sonar>)))
  "Returns md5sum for a message object of type '<Sonar>"
  #x32a7fd24a5630b5643e0d1882893197a)
(defmethod message-definition ((type (eql '<Sonar>)))
  "Returns full string definition for message of type '<Sonar>"
  (format nil "Header header~%float32 dist~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Sonar>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
))
(defmethod ros-message-to-list ((msg <Sonar>))
  "Converts a ROS message object to a list"
  (list '<Sonar>
    (cons ':header (header-val msg))
    (cons ':dist (dist-val msg))
))
