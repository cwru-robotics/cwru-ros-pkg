# abb_common

Wsn copied this info from swri-ros-pkg/abb.


copied irb_120.urdf to irb_120.urdf.xacro, just to spoof the launcher (which wants a .xacro file)
irb_120.xacro is not up to date with hydro (complaints about <include> syntax), but urdf seems OK

tried launching this model with: roslaunch abb_common irb120.launch

Produces nice looking model in Gazebo--until the links start to fall apart!

(Try this instead? https://github.com/ros-industrial/abb_experimental)

## Example usage
'rosrun gazebo_ros gazebo`
`roslaunch abb_common irb120.launch`

## Running tests/demos
    