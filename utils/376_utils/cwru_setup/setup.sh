#!/bin/bash

export PATH=/opt/hku_drc_class_scripts/netbeans:$PATH

# source ROS
. /opt/ros/hydro/setup.bash

# source drcsim and gazebo
# prefer local compiled copies over distributed binaries
#if [ -f /usr/local/share/drcsim/setup.sh ]; then
#    . /usr/local/share/drcsim/setup.sh
#else
#    . /usr/share/drcsim/setup.sh
#fi

alias hku_make='source /opt/hku_drc_class_scripts/hku_make.sh'
alias hku_clean='source /opt/hku_drc_class_scripts/hku_clean.sh'
alias hku_create_pkg='source /opt/hku_drc_class_scripts/hku_create_pkg.sh'
alias cwru_make='source /opt/hku_drc_class_scripts/hku_make.sh'
alias cwru_clean='source /opt/hku_drc_class_scripts/hku_clean.sh'
alias cwru_create_pkg='source /opt/hku_drc_class_scripts/hku_create_pkg.sh'

# setup the ros workspace ... once
export ROS_WORKSPACE=~/ros_workspace

if [ -n "$HKU_WORKSPACE" ]; then
  export ROS_WORKSPACE=$HKU_WORKSPACE
fi

export CLASS_REPO_DIR=$(find $ROS_WORKSPACE -maxdepth 2 -name '.hku_repository' | xargs -I '{}' dirname {} | head -n 1)

# Source the catkin workspace setup if we can find it

if [ -f $CLASS_REPO_DIR/catkin/devel/setup.sh ]; then
  . $CLASS_REPO_DIR/catkin/devel/setup.sh
fi

# setup the ROS_HOSTNAME and ROS_MASTER_URI
export ROS_HOSTNAME=`hostname`
export ROS_MASTER_URI=http://`hostname`:11311

# setup the ros workspace and package path ... again
export ROS_WORKSPACE=~/ros_workspace

if [ -n "$HKU_WORKSPACE" ]; then
  export ROS_WORKSPACE=$HKU_WORKSPACE
fi

export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH # might need to restrict this to the rosbuild folder

if [ -n "$HKU_PACKAGE_PATH" ]; then
  export ROS_PACKAGE_PATH=$HKU_PACKAGE_PATH:$ROS_PACKAGE_PATH
fi

# these are used for the psmove libraries
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib64/
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib64/pkgconfig/

# multisense libs
export LD_LIBRARY_PATH=/opt/ros/hydro/share/multisense_ros_driver/multisense_ros/lib:/opt/ros/hydro/share/multisense_ros_driver/multisense_lib/lib:$LD_LIBRARY_PATH

# Allow custom Gazebo Models to be used
#if [[ ! -z $CLASS_REPO_DIR ]]; then
#  if [ $(rospack find hku_models) ]; then
#    export GAZEBO_MODEL_PATH=$(rospack find hku_models):$GAZEBO_MODEL_PATH
#  fi
#fi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH

# Qwt 6.1 libs
export QWT_ROOT=/usr/share/qwt6-6.1.0
export QT_PLUGIN_PATH="${QWT_ROOT}/plugins:$QT_PLUGIN_PATH"

# useful for changing the ros-master
rosmaster() 
{
    if [[ $# -eq 0 ]]; then
	echo $ROS_MASTER_URI
    elif [[ $# -eq 1 ]]; then
	export ROS_MASTER_URI=http://$1:11311
	echo $ROS_MASTER_URI
    elif [[ $# -eq 2 ]]; then
	export ROS_MASTER_URI=http://$1:$2
	echo $ROS_MASTER_URI
    else
	echo "Usage:"
	echo "    rosmaster <hostname> [<port>] - if left out <port> defaults to 11311"
    fi
}

# list and change the hostname without an ugly export
roshostname()
{
    if [[ $# -eq 0 ]]; then
	echo $ROS_HOSTNAME
    elif [[ $# -eq 1 ]]; then
	export ROS_HOSTNAME=$1
	echo $ROS_HOSTNAME
    else
	echo "Usage:"
	echo "    roshostname <hostname>"
    fi
}

# Set up completions

function _hku_make {
  local arg
  COMPREPLY=()
  arg="${COMP_WORDS[COMP_CWORD]}"
  _roscomplete
  if [[ ${arg} =~ \-\-.* ]]; then
    COMPREPLY=(${COMPREPLY[@]} $(compgen -W "-D" -- ${arg}))
  fi
}

function _hku_clean {
  local arg
  COMPREPLY=()
  arg="${COMP_WORDS[COMP_CWORD]}"
  _roscomplete
  if [[ ${arg} =~ \-\-.* ]]; then
    COMPREPLY=(${COMPREPLY[@]} $(compgen -W "-D" -- ${arg}))
  fi
}

function _hku_create_pkg {
  local arg
  COMPREPLY=()
  arg="${COMP_WORDS[COMP_CWORD]}"
  _roscomplete
}

complete -F "_hku_make" "hku_make"
complete -F "_hku_clean" "hku_clean"
complete -F "_hku_create_pkg" "hku_create_pkg"
complete -F "_hku_make" "cwru_make"
complete -F "_hku_clean" "cwru_clean"
complete -F "_hku_create_pkg" "cwru_create_pkg"
