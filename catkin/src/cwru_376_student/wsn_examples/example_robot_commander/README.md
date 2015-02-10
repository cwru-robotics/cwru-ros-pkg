# example_robot_commander

Here are example nodes that publish values on the "cmd_vel" topic.
The nodes are specialized to send to topic robot0/cmd_vel, which works with the STDR simulator.
Change this topic to jinx/cmd_vel to drive the robot "Jinx" in the lab.


The version "vel_scheduler" is reactive.  It ramps velocity up and down and will recover
from halts.  To do so, it uses odometry info, published by STDR on topic /robot0/odom.
To run, e.g., on Jinx, change this topic to listen to Jinx's odom messages.

The versions "interactive_robot_commander" and "interactive_robot_commander_v2" do not perform velocity profiling.  However, they do respond to interactive motion commands.  For "interactive_robot_commander", the input is an interactive marker (created by package/node: example_interactive_marker/IM_example3), which publishes poses to topic "/path_end/update"."  The interactive robot commander subscribes to this topic.  However, it does not accept in motion input commands until a trigger is induced.  The trigger action is through a ROS service within the interactive commander.  One can invoke the trigger manually from a command line with:
`rosservice call trigger_path_goal 1`
The motion commander will cause the robot to perform a sequence of 3 actions: 1) turn towards the new goal position; 2) travel at constant velocity towards the new goal position; and 3) rotate with spin-in-place to align the robot heading with the commanded heading.

Version 2 of the interactive commander is somewhat more complex, since it uses a transform listener and looks for transforms with respect to a map.  It also can respond to the "2D Nav Goal" tool in Rviz, and it does not require a separate action to trigger the motion.

## Example usage
To run the STDR simulator:  
'roslaunch cwru_376_launchers stdr_glennan_2.launch'
Then run a velocity commander, e.g.:
'rosrun example_robot_commander vel_scheduler'
Can also observe the speed commands by plotting using:
rqt_plot /robot0/cmd_vel/linear/x

To run cwruBot with interactive motion commands:
1) start roscore
2) `rosrun gazebo_ros gazebo`
3) `roslaunch cwru_urdf cwruBot.launch`
optionally, insert a Gazebo model, e.g. the starting pen
4) `rosrun example_interactive_marker IM_example3`
5) `rosrun example_robot_commander interactive_robot_commander`
6) `rosrun rviz rviz`
make sure an interactive-marker item is included, with the topic set to /path_end/update.

One can now move the interactive marker, then trigger motion with the command:
`rosservice call trigger_path_goal 1`






    
