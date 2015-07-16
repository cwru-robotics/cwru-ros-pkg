# cwru-ros-pkg-hydro
Welcome to cwru-ros-pkg-hydro.  This is the latest release for Case Western Reserve University's Mobile / Dexterous Robotics Lab's ROS package to run on all of the lab's mobile robots.  

#### People
Originally developed by [Eric Perko](https://github.com/ericperko) and [Chad Rockey](https://github.com/chadrockey) with contributions by [Jesse Fish](https://github.com/erebuswolf) and [Ed Venator](https://github.com/evenator).  Currently maintained by [Dr. Wyatt Newman](https://github.com/wsnewman) and [Luc Bettaieb](https://github.com/lucbettaieb).

#### EECS 376/476: Mobile Robotics
Your forked repositories should sync with updates from this repository from now on.

#### Current ROS Dependencies
Our package will be continuously updated as more functionality is added.  Please make sure your development system (and whatever robot you're trying to run our package on) has all of the dependencies listed below.  You should be able to install the dependencies by running `sudo apt-get install (name of dependency)`
* ros-hydro-joy (for cwru_teleop)
* ros-hydro-sound-play

##### Also Depends on catkin_simple
* inside utils/376_utils, do `sudo dpkg -i catkin...`
