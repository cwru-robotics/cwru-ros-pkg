; Auto-generated. Do not edit!


(in-package costmap_2d-msg)


;//! \htmlinclude VoxelGrid.msg.html

(defclass <VoxelGrid> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (data
    :reader data-val
    :initarg :data
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (origin
    :reader origin-val
    :initarg :origin
    :type geometry_msgs-msg:<Point32>
    :initform (make-instance 'geometry_msgs-msg:<Point32>))
   (resolutions
    :reader resolutions-val
    :initarg :resolutions
    :type geometry_msgs-msg:<Vector3>
    :initform (make-instance 'geometry_msgs-msg:<Vector3>))
   (size_x
    :reader size_x-val
    :initarg :size_x
    :type integer
    :initform 0)
   (size_y
    :reader size_y-val
    :initarg :size_y
    :type integer
    :initform 0)
   (size_z
    :reader size_z-val
    :initarg :size_z
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <VoxelGrid>) ostream)
  "Serializes a message object of type '<VoxelGrid>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'data))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'data))
  (serialize (slot-value msg 'origin) ostream)
  (serialize (slot-value msg 'resolutions) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'size_x)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'size_x)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'size_x)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'size_x)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'size_y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'size_y)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'size_y)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'size_y)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'size_z)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'size_z)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'size_z)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'size_z)) ostream)
)
(defmethod deserialize ((msg <VoxelGrid>) istream)
  "Deserializes a message object of type '<VoxelGrid>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'data) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'data)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (deserialize (slot-value msg 'origin) istream)
  (deserialize (slot-value msg 'resolutions) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'size_x)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'size_x)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'size_x)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'size_x)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'size_y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'size_y)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'size_y)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'size_y)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'size_z)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'size_z)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'size_z)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'size_z)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<VoxelGrid>)))
  "Returns string type for a message object of type '<VoxelGrid>"
  "costmap_2d/VoxelGrid")
(defmethod md5sum ((type (eql '<VoxelGrid>)))
  "Returns md5sum for a message object of type '<VoxelGrid>"
  "48a040827e1322073d78ece5a497029c")
(defmethod message-definition ((type (eql '<VoxelGrid>)))
  "Returns full string definition for message of type '<VoxelGrid>"
  (format nil "Header header~%uint32[] data~%geometry_msgs/Point32 origin~%geometry_msgs/Vector3 resolutions~%uint32 size_x~%uint32 size_y~%uint32 size_z~%~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(defmethod serialization-length ((msg <VoxelGrid>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (reduce #'+ (slot-value msg 'data) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     (serialization-length (slot-value msg 'origin))
     (serialization-length (slot-value msg 'resolutions))
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <VoxelGrid>))
  "Converts a ROS message object to a list"
  (list '<VoxelGrid>
    (cons ':header (header-val msg))
    (cons ':data (data-val msg))
    (cons ':origin (origin-val msg))
    (cons ':resolutions (resolutions-val msg))
    (cons ':size_x (size_x-val msg))
    (cons ':size_y (size_y-val msg))
    (cons ':size_z (size_z-val msg))
))
