#! /bin/bash

roslaunch `rosstack find cwru_semi_stable`/cwru_bringup_no_tele.launch &

sleep 5
roslaunch `rospack find cwru_nav`/start_tour_guide_nav.launch &

sleep 1
rosrun rviz rviz &

sleep 5

roslaunch `rospack find tour_guide_executive`/seed_initial_pose.launch &

sleep 5
roslaunch `rospack find tour_guide_executive`/start_executive.launch &

wait

