motor_controller
===============



ROS stack for interfacing with Robotis Dynamixel MX-64 servo motor.
Aimed at controlling Abby gripper in Case Western Reserve University.

Need Python compiler!!
Any subversion of Python 2 is okay. Not tested on Python 3.

use:
In terminal:
	roslaunch motor_controller controller_manager.launch
In another terminal:
	roslaunch motor_controller tilt_controller.launch
Then:
Spread hand:
	rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -3.5
Grab object:
	rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -4.0

DO NOT CHANGE THE VALUE!! IT MAY BREAK GRIPPER!

Default mount point for usb2dynamixel is at /dev/ttyUSB0. You can change this in controller_manager.launch

Motor ID is constraint within 1 to 20. This could be modified in the same file.

An Zhigang Apr. 20, 2015
Revision: An Zhigang Apr. 29, 2015
Revision: An Zhigang Apr. 30, 2015

If you have any further questions, please contact me through email:
anzhg@icloud.com
or:
zxa41@case.edu
