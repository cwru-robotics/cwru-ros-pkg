# example_robot_commander

Here are example nodes that publish values on the "cmd_vel" topic.
The nodes are specialized to send to topic robot0/cmd_vel, which works with the STDR simulator.
Change this topic to jinx/cmd_vel to drive the robot "Jinx" in the lab.


The version "vel_scheduler" is reactive.  It ramps velocity up and down and will recover
from halts.  To do so, it uses odometry info, published by STDR on topic /robot0/odom.
To run, e.g., on Jinx, change this topic to listen to Jinx's odom messages.

## Example usage
To run the STDR simulator:  
'roslaunch cwru_376_launchers stdr_glennan_2.launch'
Then run a velocity commander, e.g.:
'rosrun example_robot_commander vel_scheduler'
Can also observe the speed commands by plotting using:
rqt_plot /robot0/cmd_vel/linear/x


    