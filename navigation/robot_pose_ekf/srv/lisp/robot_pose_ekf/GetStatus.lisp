; Auto-generated. Do not edit!


(in-package robot_pose_ekf-srv)


;//! \htmlinclude GetStatus-request.msg.html

(defclass <GetStatus-request> (ros-message)
  ()
)
(defmethod serialize ((msg <GetStatus-request>) ostream)
  "Serializes a message object of type '<GetStatus-request>"
)
(defmethod deserialize ((msg <GetStatus-request>) istream)
  "Deserializes a message object of type '<GetStatus-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GetStatus-request>)))
  "Returns string type for a service object of type '<GetStatus-request>"
  "robot_pose_ekf/GetStatusRequest")
(defmethod md5sum ((type (eql '<GetStatus-request>)))
  "Returns md5sum for a message object of type '<GetStatus-request>"
  "4fe5af303955c287688e7347e9b00278")
(defmethod message-definition ((type (eql '<GetStatus-request>)))
  "Returns full string definition for message of type '<GetStatus-request>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <GetStatus-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GetStatus-request>))
  "Converts a ROS message object to a list"
  (list '<GetStatus-request>
))
;//! \htmlinclude GetStatus-response.msg.html

(defclass <GetStatus-response> (ros-message)
  ((status
    :reader status-val
    :initarg :status
    :type string
    :initform ""))
)
(defmethod serialize ((msg <GetStatus-response>) ostream)
  "Serializes a message object of type '<GetStatus-response>"
  (let ((__ros_str_len (length (slot-value msg 'status))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'status))
)
(defmethod deserialize ((msg <GetStatus-response>) istream)
  "Deserializes a message object of type '<GetStatus-response>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'status) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'status) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<GetStatus-response>)))
  "Returns string type for a service object of type '<GetStatus-response>"
  "robot_pose_ekf/GetStatusResponse")
(defmethod md5sum ((type (eql '<GetStatus-response>)))
  "Returns md5sum for a message object of type '<GetStatus-response>"
  "4fe5af303955c287688e7347e9b00278")
(defmethod message-definition ((type (eql '<GetStatus-response>)))
  "Returns full string definition for message of type '<GetStatus-response>"
  (format nil "string status~%~%"))
(defmethod serialization-length ((msg <GetStatus-response>))
  (+ 0
     4 (length (slot-value msg 'status))
))
(defmethod ros-message-to-list ((msg <GetStatus-response>))
  "Converts a ROS message object to a list"
  (list '<GetStatus-response>
    (cons ':status (status-val msg))
))
(defmethod service-request-type ((msg (eql 'GetStatus)))
  '<GetStatus-request>)
(defmethod service-response-type ((msg (eql 'GetStatus)))
  '<GetStatus-response>)
(defmethod ros-datatype ((msg (eql 'GetStatus)))
  "Returns string type for a service object of type '<GetStatus>"
  "robot_pose_ekf/GetStatus")
