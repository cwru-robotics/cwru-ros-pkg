; Auto-generated. Do not edit!


(in-package navfn-srv)


;//! \htmlinclude SetCostmap-request.msg.html

(defclass <SetCostmap-request> (ros-message)
  ((costs
    :reader costs-val
    :initarg :costs
    :type (vector fixnum)
   :initform (make-array 0 :element-type 'fixnum :initial-element 0))
   (height
    :reader height-val
    :initarg :height
    :type fixnum
    :initform 0)
   (width
    :reader width-val
    :initarg :width
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <SetCostmap-request>) ostream)
  "Serializes a message object of type '<SetCostmap-request>"
  (let ((__ros_arr_len (length (slot-value msg 'costs))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'costs))
    (write-byte (ldb (byte 8 0) (slot-value msg 'height)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'height)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'width)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'width)) ostream)
)
(defmethod deserialize ((msg <SetCostmap-request>) istream)
  "Deserializes a message object of type '<SetCostmap-request>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'costs) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'costs)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'height)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'width)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'width)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<SetCostmap-request>)))
  "Returns string type for a service object of type '<SetCostmap-request>"
  "navfn/SetCostmapRequest")
(defmethod md5sum ((type (eql '<SetCostmap-request>)))
  "Returns md5sum for a message object of type '<SetCostmap-request>"
  "370ec969cdb71f9cde7c7cbe0d752308")
(defmethod message-definition ((type (eql '<SetCostmap-request>)))
  "Returns full string definition for message of type '<SetCostmap-request>"
  (format nil "uint8[] costs~%uint16 height~%uint16 width~%~%"))
(defmethod serialization-length ((msg <SetCostmap-request>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'costs) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
     2
     2
))
(defmethod ros-message-to-list ((msg <SetCostmap-request>))
  "Converts a ROS message object to a list"
  (list '<SetCostmap-request>
    (cons ':costs (costs-val msg))
    (cons ':height (height-val msg))
    (cons ':width (width-val msg))
))
;//! \htmlinclude SetCostmap-response.msg.html

(defclass <SetCostmap-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SetCostmap-response>) ostream)
  "Serializes a message object of type '<SetCostmap-response>"
)
(defmethod deserialize ((msg <SetCostmap-response>) istream)
  "Deserializes a message object of type '<SetCostmap-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SetCostmap-response>)))
  "Returns string type for a service object of type '<SetCostmap-response>"
  "navfn/SetCostmapResponse")
(defmethod md5sum ((type (eql '<SetCostmap-response>)))
  "Returns md5sum for a message object of type '<SetCostmap-response>"
  "370ec969cdb71f9cde7c7cbe0d752308")
(defmethod message-definition ((type (eql '<SetCostmap-response>)))
  "Returns full string definition for message of type '<SetCostmap-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <SetCostmap-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SetCostmap-response>))
  "Converts a ROS message object to a list"
  (list '<SetCostmap-response>
))
(defmethod service-request-type ((msg (eql 'SetCostmap)))
  '<SetCostmap-request>)
(defmethod service-response-type ((msg (eql 'SetCostmap)))
  '<SetCostmap-response>)
(defmethod ros-datatype ((msg (eql 'SetCostmap)))
  "Returns string type for a service object of type '<SetCostmap>"
  "navfn/SetCostmap")
