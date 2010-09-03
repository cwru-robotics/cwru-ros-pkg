; Auto-generated. Do not edit!


(in-package base_local_planner-msg)


;//! \htmlinclude CmdVel_viz.msg.html

(defclass <CmdVel_viz> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (path_dist_cost
    :reader path_dist_cost-val
    :initarg :path_dist_cost
    :type float
    :initform 0.0)
   (goal_dist_cost
    :reader goal_dist_cost-val
    :initarg :goal_dist_cost
    :type float
    :initform 0.0)
   (obs_dist_cost
    :reader obs_dist_cost-val
    :initarg :obs_dist_cost
    :type float
    :initform 0.0)
   (heading_diff_cost
    :reader heading_diff_cost-val
    :initarg :heading_diff_cost
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <CmdVel_viz>) ostream)
  "Serializes a message object of type '<CmdVel_viz>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'path_dist_cost))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'goal_dist_cost))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'obs_dist_cost))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'heading_diff_cost))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <CmdVel_viz>) istream)
  "Deserializes a message object of type '<CmdVel_viz>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'path_dist_cost) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'goal_dist_cost) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'obs_dist_cost) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'heading_diff_cost) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<CmdVel_viz>)))
  "Returns string type for a message object of type '<CmdVel_viz>"
  "base_local_planner/CmdVel_viz")
(defmethod md5sum ((type (eql '<CmdVel_viz>)))
  "Returns md5sum for a message object of type '<CmdVel_viz>"
  "7bab164ed55c77329ea34c4dc7c6a354")
(defmethod message-definition ((type (eql '<CmdVel_viz>)))
  "Returns full string definition for message of type '<CmdVel_viz>"
  (format nil "Header header~%~%float32 path_dist_cost~%float32 goal_dist_cost~%float32 obs_dist_cost~%float32 heading_diff_cost~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <CmdVel_viz>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <CmdVel_viz>))
  "Converts a ROS message object to a list"
  (list '<CmdVel_viz>
    (cons ':header (header-val msg))
    (cons ':path_dist_cost (path_dist_cost-val msg))
    (cons ':goal_dist_cost (goal_dist_cost-val msg))
    (cons ':obs_dist_cost (obs_dist_cost-val msg))
    (cons ':heading_diff_cost (heading_diff_cost-val msg))
))
