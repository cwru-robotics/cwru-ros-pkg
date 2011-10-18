#! /bin/bash

roslaunch `rosstack find cwru_semi_stable`/cwru_bringup_no_tele.launch &

sleep 5
roslaunch `rospack find cwru_nav`/start_tour_guide_nav.launch &

sleep 1
rosrun rviz rviz -d ~/Documents/Bluebash_lidar_config.vcg &

sleep 1

rosrun rviz rviz -d ~/Documents/bluebash_kinect_config.vcg &


sleep 5
#roslaunch `rospack find tour_guide_bluebash`/start_bluebash.launch &

wait

