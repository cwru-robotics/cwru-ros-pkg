# cwru_urdf
Here is a simple model of the CWRU wheelchair-based mobile robots.  The geometric modeling is low resolution, but adequate.  This version includes simulated LIDAR via a gazebo plugin and differential steering via another gazebo plugin.  

The cwruBot accepts speed/spin commands via topic cmd_vel.  It outputs odometry on the /odom topic and simulated LIDAR on the /laser/scan topic.  

Inertias, max wheel torque and max speeds should not be trusted.

The robot will stall if it runs into a heavy object--but it is able to push around lighter objects (e.g. construction cones).

## Example usage
In a terminal, run `roscore`

In another terminal run `rosrun gazebo_ros gazebo`  

This should produce a flat, gray ground-plane.  If this does not come up, kill this process and restart it.

In another terminal, run:
 `roslaunch cwru_urdf cwruBot.launch`

In rviz (`rosrun rviz rviz`), choose "base_link" as the fixed frame in rviz. 
Add a display of LaserScan (on topic /laser/scan) to visualize the lidar.
Add a topic of "DepthCloud" and set the topic to /kinect/depth/image_raw to see the point cloud from simulated Kinect sensor.
Add a topic of "Camera" and set the topic to /kinect/rgb/image_raw to see video from simulation of the Kinect's color camera.

Can test run the robot from a command line, e.g., with (e.g.):

`rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.1}}'`

To cause the robot to move at 0.2m/s and rotate at 0.1rad/sec, making slow circles.
Run `rostopic echo odom` to see the robot achieve the commanded speeds, and to see the pose evolve as the robot moves.

Add models to gazebo (e.g., construction barrel) to see the LIDAR, camera and kinect visualizations.

    
