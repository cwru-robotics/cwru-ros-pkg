#!/bin/bash

notify(){
  which notify-send > /dev/null
  notify_present=$?

  which fortune > /dev/null
  fortune_present=$?

  which cowsay > /dev/null
  cowsay_present=$?

  if [ $# -lt 3 ]; then
    echo "Notify called with incorrect arguments"
    return 1
  fi

  echo "$2"
  if [ $1 -eq 0 ]; then
    if [ $notify_present -eq 0 ]; then
      if [ $cowsay_present -eq 0 ]; then
        notify-send --hint=int:transient:1 --icon=emblem-urgent "$2" "$(cowsay $3)"
        echo "$(cowsay $3)"
      else
        notify-send --hint=int:transient:1 --icon=emblem-urgent "$2" "$3"
        echo "$3"
      fi
    fi
  else
    if [ $notify_present -eq 0 ]; then
      if [ $fortune_present -eq 0 ]; then
        if [ $cowsay_present -eq 0 ]; then
          notify-send --hint=int:transient:1 --icon=emblem-important "$2" "$3 $(cowsay $(fortune))"
          echo "$3 $(cowsay $(fortune))"
        else
          notify-send --hint=int:transient:1 --icon=emblem-important "$2" "$3 $(fortune)"
          echo "$3 $(fortune)"
        fi
      else
        notify-send --hint=int:transient:1 --icon=emblem-important "$2" "$3"
        echo "$3"
      fi
    fi
  fi

  return $1
}

getTeamRepo(){
  team_repo_dir=$(find $ROS_WORKSPACE -maxdepth 2 -name '.hku_repository' | xargs -I '{}' dirname {} | head -n 1)
  if [ -z $team_repo_dir ]; then
    notify 1 "Could not find team repo" ""
    return 1
  fi
  return 0
}

isCatkinPackage(){
  if [ `find $1/catkin/src/ -type d -name $2 | wc -l` -gt 0 ]; then
    return 0
  else
    return 1
  fi
}

catkinBuild(){
  # These lines will create a bash array of dependencies for the packages
  # specified. This has to happen before changing environment variables
  dep_packages=()
  if [ $# -ge 3 ]; then
    for pkg in ${@:3}; do
      dep_packages=($(for dep_pkg in "${dep_packages[@]}" "$(rospack deps $pkg)"; do
        echo "$dep_pkg";
      done | sort -du))
    done

    echo "Dependencies:"
    echo "${dep_packages[@]}"
  fi

  unset CMAKE_PREFIX_PATH
  ROS_SAVED_WORKSPACE=$ROS_WORKSPACE
  SAVED_GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH

  . /opt/ros/groovy/setup.sh
  . /usr/share/drcsim/setup.sh

  echo "number of arguments $#"
  echo "arguments received $@"

  if [ ! -z $1/catkin/src/CmakeLists.txt ]; then
    cd $1/catkin

    if [ $# -ge 3 ]; then
      echo "building with deps"
      # build with deps
      if $2; then
        catkin_make -DCATKIN_WHITELIST_PACKAGES=$(echo "${dep_packages[@]}" "${@:3}" | tr ' ' ';') --pkg "${dep_packages[@]}" "${@:3}"
      else
        echo $(echo "${dep_packages[@]}" "${@:3}" | tr ' ' ';')
        catkin_make -DCATKIN_WHITELIST_PACKAGES=$(echo "${dep_packages[@]}" "${@:3}" | tr ' ' ';') --pkg "${@:3}"
      fi
    else
      catkin_make --cmake-args -UCATKIN_WHITELIST_PACKAGES
    fi

    catkin_build_success=$?

    if [ $catkin_build_success -ne 0 ]; then
      notify 1 "Failed to build catkin packages" ""
    else
      notify 0 "make ${@:3}" "Success!"
    fi

    # NOTE cswetenham Source even if we failed to build some packages, in case it's useful - I think it's needed for
    # roscd to work
    . $team_repo_dir/catkin/devel/setup.sh

  else
    echo "WARNING: No catkin workspace found"
  fi

  export ROS_WORKSPACE=$ROS_SAVED_WORKSPACE
  export GAZEBO_MODEL_PATH=$SAVED_GAZEBO_MODEL_PATH
  export ROS_PACKAGE_PATH=$1:$ROS_PACKAGE_PATH

  return $catkin_build_success
}

# this could probably be refactored and
# merged with rosbuild since 99% of the function
# is the same, but for now I'm keeping it separate
rosbuildWithDeps(){

  export ROS_PACKAGE_PATH=$1:$ROS_PACKAGE_PATH

  pushd . > /dev/null
  if [ -z $2 ]; then
    echo "ERROR: You must specify at least one package name when building with dependencies" >&2
    return 1
  fi

  rosmake --robust "${@:2}"
  rosbuild_success=$?

  popd > /dev/null

  if [ $rosbuild_success -eq 0 ]; then
    notify 0 "make ${@:2}" 'Success!'
  else
    notify 1 "make ${@:2}" 'Failure :('
  fi

  return $rosbuild_success
}

rosbuild(){
  export ROS_PACKAGE_PATH=$1:$ROS_PACKAGE_PATH

  pushd . > /dev/null
  if [ -z $2 ]; then
    rosbuild_target="all"
    rosmake --robust $(find $1 -name 'manifest.xml' | xargs -I '{}' dirname {} | xargs -I '{}' basename {} | tr '\n' ' ')
  else
    for pkg in ${@:2}; do
      rosbuild_target=$pkg
      cd `rospack find $pkg` && make
    done
  fi
  rosbuild_success=$?
  popd > /dev/null

  if [ $rosbuild_success -eq 0 ]; then
    notify 0 "make $rosbuild_target" 'Success!'
  else
    notify 1 "make $rosbuild_target" 'Failure :('
  fi

  return $rosbuild_success
}

catkinClean() {
  if [ -z $2 ]; then
    if [ -d "$1/catkin/build" ]; then
      rm -r "$1/catkin/build" || return $?
    fi

    if [ -d "$1/catkin/devel" ]; then
      rm -r "$1/catkin/devel" || return $?
    fi

    if [ -d "$1/catkin/install" ]; then
      rm -r "$1/catkin/install" || return $?
    fi
  else
    find "$1/catkin/build" -type d -name $2 -exec rm -r {} \;
  fi
  return 0
}

rosbuildClean() {
  export ROS_PACKAGE_PATH=$1:$ROS_PACKAGE_PATH

  pushd .
  if [ -z $2 ]; then
    rosmake --target=clean --robust $(find $1 -name 'manifest.xml' | xargs -I '{}' dirname {} | xargs -I '{}' basename {} | tr '\n' ' ')
  else
    cd `rospack find $2` && make clean
  fi
  rosbuild_success=$?
  popd

  return $rosbuild_success
}


