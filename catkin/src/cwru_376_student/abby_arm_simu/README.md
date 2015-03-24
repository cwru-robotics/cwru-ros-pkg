# abby_arm_simu

Gazebo model of ABB IRB120, wsn, March, 2015.  
Based on rrbot tutorial in ROS, plus incorporation of modeling work by Ed Venator.
Incorporates controllers, plus an emulation of a ROS-industrial interface, which receives and executes trajectories.

## Example usage
`roslaunch abby_gazebo abby_world.launch` 

This brings up the Gazebo simulation, ready to accept ROS-I style trajectory commands

## Running tests/demos
in another window,
`rosrun example_joint_space_planner`
or
`rosrun example_robot_interface test_ik_traj_sender`


    
