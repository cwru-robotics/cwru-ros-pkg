#! /bin/bash

export ROBOT=roberto

roslaunch `rospack find bk_planner`/start_bk_nav.launch &

sleep 10

roslaunch `rospack find tour_guide_executive`/start_executive.launch &

wait
