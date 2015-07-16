#Reference frame that the following coordinates should be interpreted relative to and timestamp to use when transforming
Header header

#Segment type enums
int8 LINE = 1
int8 ARC = 2
int8 SPIN_IN_PLACE = 3

#Segment type
int8 seg_type

#Segment number
uint32 seg_number

#Segment length
float64 seg_length

#Reference point
geometry_msgs/Point ref_point

#Initial tangent angle
geometry_msgs/Quaternion init_tan_angle

#Curvature
float64 curvature

#Speed limits for this segment
geometry_msgs/Twist max_speeds
geometry_msgs/Twist min_speeds

#Acceleration limit for this segment (m/s^2 for lines/arcs, rads/s^2 for spin-in-place)
float64 accel_limit

#Deceleration limit for this segment
float64 decel_limit
