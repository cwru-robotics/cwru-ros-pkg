# abby_wsn_dev
* Author: Wyatt Newman; March, 2015
# based on modification of: gazebo_ros_demos
* Author: Dave Coleman <davetcoleman@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)
* Inception Date: 4 June 2013
* Version: 1.0.0

Example robots and code for interfacing Gazebo with ROS

## Documentation and Tutorials
rrbot tutorial is here:
[On gazebosim.org](http://gazebosim.org/tutorials?cat=connect_ros)

Wsn modifications for ABB IRB120 model:
Start up as follows:
*roslaunch rrbot_gazebo rrbot_world.launch 

Then the IRB120 model is ready to accept ROS-I style trajectory commands.
E.g., try running:
*rosrun example_robot_interface test_ik_traj_sender
This will send the robot to 8 different IK solutions of the same commanded tool-flange pose

*or try: rosrun example_joint_space_planner test_ik_traj_sender2
which will cause the arm to move through a smooth trajectory parallel to the world y-axis






