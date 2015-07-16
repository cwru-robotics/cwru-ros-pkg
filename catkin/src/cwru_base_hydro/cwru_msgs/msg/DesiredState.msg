#Standard ROS header for timestamp and frame_id
Header header

#Segment type
int8 seg_type

#Segment number
uint32 seg_number

#6DOF pose
geometry_msgs/Pose des_pose

#Desired speed
float64 des_speed

#Desired curvature
float64 des_rho

#Desired distance along the segment
float64 des_lseg
