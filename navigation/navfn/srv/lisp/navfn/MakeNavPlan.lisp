; Auto-generated. Do not edit!


(in-package navfn-srv)


;//! \htmlinclude MakeNavPlan-request.msg.html

(defclass <MakeNavPlan-request> (ros-message)
  ((start
    :reader start-val
    :initarg :start
    :type geometry_msgs-msg:<PoseStamped>
    :initform (make-instance 'geometry_msgs-msg:<PoseStamped>))
   (goal
    :reader goal-val
    :initarg :goal
    :type geometry_msgs-msg:<PoseStamped>
    :initform (make-instance 'geometry_msgs-msg:<PoseStamped>)))
)
(defmethod serialize ((msg <MakeNavPlan-request>) ostream)
  "Serializes a message object of type '<MakeNavPlan-request>"
  (serialize (slot-value msg 'start) ostream)
  (serialize (slot-value msg 'goal) ostream)
)
(defmethod deserialize ((msg <MakeNavPlan-request>) istream)
  "Deserializes a message object of type '<MakeNavPlan-request>"
  (deserialize (slot-value msg 'start) istream)
  (deserialize (slot-value msg 'goal) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<MakeNavPlan-request>)))
  "Returns string type for a service object of type '<MakeNavPlan-request>"
  "navfn/MakeNavPlanRequest")
(defmethod md5sum ((type (eql '<MakeNavPlan-request>)))
  "Returns md5sum for a message object of type '<MakeNavPlan-request>"
  "8ffef29bc8b086289124c16a8daa989d")
(defmethod message-definition ((type (eql '<MakeNavPlan-request>)))
  "Returns full string definition for message of type '<MakeNavPlan-request>"
  (format nil "geometry_msgs/PoseStamped start~%geometry_msgs/PoseStamped goal~%~%"))
(defmethod serialization-length ((msg <MakeNavPlan-request>))
  (+ 0
     (serialization-length (slot-value msg 'start))
     (serialization-length (slot-value msg 'goal))
))
(defmethod ros-message-to-list ((msg <MakeNavPlan-request>))
  "Converts a ROS message object to a list"
  (list '<MakeNavPlan-request>
    (cons ':start (start-val msg))
    (cons ':goal (goal-val msg))
))
;//! \htmlinclude MakeNavPlan-response.msg.html

(defclass <MakeNavPlan-response> (ros-message)
  ((plan_found
    :reader plan_found-val
    :initarg :plan_found
    :type fixnum
    :initform 0)
   (error_message
    :reader error_message-val
    :initarg :error_message
    :type string
    :initform "")
   (path
    :reader path-val
    :initarg :path
    :type (vector geometry_msgs-msg:<PoseStamped>)
   :initform (make-array 0 :element-type 'geometry_msgs-msg:<PoseStamped> :initial-element (make-instance 'geometry_msgs-msg:<PoseStamped>))))
)
(defmethod serialize ((msg <MakeNavPlan-response>) ostream)
  "Serializes a message object of type '<MakeNavPlan-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'plan_found)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'error_message))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'error_message))
  (let ((__ros_arr_len (length (slot-value msg 'path))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'path))
)
(defmethod deserialize ((msg <MakeNavPlan-response>) istream)
  "Deserializes a message object of type '<MakeNavPlan-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'plan_found)) (read-byte istream))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'error_message) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'error_message) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'path) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'path)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance 'geometry_msgs-msg:<PoseStamped>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<MakeNavPlan-response>)))
  "Returns string type for a service object of type '<MakeNavPlan-response>"
  "navfn/MakeNavPlanResponse")
(defmethod md5sum ((type (eql '<MakeNavPlan-response>)))
  "Returns md5sum for a message object of type '<MakeNavPlan-response>"
  "8ffef29bc8b086289124c16a8daa989d")
(defmethod message-definition ((type (eql '<MakeNavPlan-response>)))
  "Returns full string definition for message of type '<MakeNavPlan-response>"
  (format nil "~%uint8 plan_found~%string error_message~%~%# if plan_found is true, this is an array of waypoints from start to goal, where the first one equals start and the last one equals goal~%geometry_msgs/PoseStamped[] path~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(defmethod serialization-length ((msg <MakeNavPlan-response>))
  (+ 0
     1
     4 (length (slot-value msg 'error_message))
     4 (reduce #'+ (slot-value msg 'path) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <MakeNavPlan-response>))
  "Converts a ROS message object to a list"
  (list '<MakeNavPlan-response>
    (cons ':plan_found (plan_found-val msg))
    (cons ':error_message (error_message-val msg))
    (cons ':path (path-val msg))
))
(defmethod service-request-type ((msg (eql 'MakeNavPlan)))
  '<MakeNavPlan-request>)
(defmethod service-response-type ((msg (eql 'MakeNavPlan)))
  '<MakeNavPlan-response>)
(defmethod ros-datatype ((msg (eql 'MakeNavPlan)))
  "Returns string type for a service object of type '<MakeNavPlan>"
  "navfn/MakeNavPlan")
