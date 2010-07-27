; Auto-generated. Do not edit!


(in-package base_local_planner-msg)


;//! \htmlinclude Position2DInt.msg.html

(defclass <Position2DInt> (ros-message)
  ((x
    :reader x-val
    :initarg :x
    :type integer
    :initform 0)
   (y
    :reader y-val
    :initarg :y
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <Position2DInt>) ostream)
  "Serializes a message object of type '<Position2DInt>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'x)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'y)) ostream)
)
(defmethod deserialize ((msg <Position2DInt>) istream)
  "Deserializes a message object of type '<Position2DInt>"
  (setf (ldb (byte 8 0) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'y)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Position2DInt>)))
  "Returns string type for a message object of type '<Position2DInt>"
  "base_local_planner/Position2DInt")
(defmethod md5sum ((type (eql '<Position2DInt>)))
  "Returns md5sum for a message object of type '<Position2DInt>"
  "3b834ede922a0fff22c43585c533b49f")
(defmethod message-definition ((type (eql '<Position2DInt>)))
  "Returns full string definition for message of type '<Position2DInt>"
  (format nil "int64 x~%int64 y~%~%"))
(defmethod serialization-length ((msg <Position2DInt>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <Position2DInt>))
  "Converts a ROS message object to a list"
  (list '<Position2DInt>
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
))
