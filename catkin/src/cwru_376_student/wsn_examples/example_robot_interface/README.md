# example_robot_interface
this node is intended to stand in for "motion_download_interface" of ROS Industrial
It receives trajectories on topic "joint_path_command", but instead of sending these to an industrial robot controller,
it breaks them up into individual trajectory points and publishes them, one at a time, to topic: "joint_point_command"
at a fixed update rate, UPDATE_RATE

Test this node with simple test node, test_traj_sender, which sends a simple, hard-coded trajectory

## Example usage
`rosrun example_robot_interface example_robot_interface`
`rosrun example_robot_interface test_traj_sender`

## Running tests/demos
Try this with simulator first:
`roslaunch abby_gazebo abby_world.launch`
`rosrun example_robot_interface test_abby_sender2 joint_states:= /abby/joint_states`
Will get prompted for joint numbers and joint angles, then robot will go to these points.
    