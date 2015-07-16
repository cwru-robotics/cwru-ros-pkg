# abby_description

This model was created by wsn, March, 2015, following the rrbot Tutorial in ROS, and incorporating
model elements from Ed Venator for Abby.  Link lengths and joint limits are correct.  Meshes for link visualizations are good.
HOWEVER, dynamic properties (inertias, transmissions, torque limits, ...) are not to be trusted.

Had to define the collision models smaller to avoid the robot blowing up.  COLLISION MODELS NEED TO BE REVISITED

The irb120 model is launched as: roslaunch abby_gazebo abby_world.launch

This also launches an emulation of a ROS-I interface, which accepts trajectory commands
    
