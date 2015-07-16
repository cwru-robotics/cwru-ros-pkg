# example_des_state_generator

This is an illustrative example that is NOT COMPLETE.  It will compile and run, but it is dynamically undesirable (no velocity profiling is implemented).

Purpose of this node is to output a (nearly) continuous stream of desired states, leading from a start pose to a goal pose.
This is done by receiving a ROS "Path" (vector of subgoal poses), and redescribing the polyline path as a queue of path "segments".
At present, segments are either line segments or spin-in-place move segments.  Should be extended to incorporate curved segments.

Velocity profiling is NOT implemented.  This needs to be done.

Also, desState generation needs to be aware-of/respond-to any system halts (e.g., E-stop, obstacle, software halt...)

The complementary node "example_path_sender" shows how to communicate a (polyline) path to example_des_state_generator.
Desired states are output at frequency UPDATE_RATE as nav_msgs::Odometry messages on topic "desState".  

See the accompanying document "Desired State Generation".

## Example usage
Can use this, e.g., with cwruBot Gazebo model.  With robot running, execute:
`rosrun example_des_state_generator des_state_generator`

The desired state generator will initialize the desired state to the current odom state, then keep republishing this desired state on topic desState.

Test desState generation by running the example path sender: 
`rosrun example_des_state_generator example_path_sender`

This will transmit (via a service call) a path consisting of 3 path subgoals (vertices).  The desState node will generate a sequence of fine-grained
intermediate poses, updated frequently, published to the desState topic.  This topic may be used by a steering algorith.

    