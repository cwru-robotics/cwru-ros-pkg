#!/bin/bash

. ./build_functions.sh

# now find the team repository
getTeamRepo || return $?

OPTIND=1
make_with_deps=false

while getopts ":d" opt; do
  case $opt in
    d)
      make_with_deps=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      ;;
  esac
done
shift $((OPTIND-1))

if [ -z $1 ]; then
  if $make_with_deps; then
    echo "-d requires at least one package name" >&2
    exit 1
  fi
  echo "Making all packages"

  catkinBuild $team_repo_dir false || return $?
  rosbuild $team_repo_dir || return $?

else
  if $make_with_deps; then
    echo "Making the following packages with dependencies"

    i_catkin=0 catkin_pkgs=()
    i_rosbuild=0 rosbuild_pkgs=()
    for pkg in $@; do
      echo $pkg
      if $(isCatkinPackage $team_repo_dir $pkg); then
        catkin_pkgs[$i_catkin]=$pkg
        ((++i_catkin))
      else
        rosbuild_pkgs[$i_rosbuild]=$pkg
        ((++i_rosbuild))
      fi
    done

    if [ $i_rosbuild -gt 0 ]; then
      # for now building all catkin packages, however,
      # it might be better to replace this with a recursive call to
      # rospack depends <package name> and then separate the catkins
      # from the non-catkins and build everything in the list
      # (essentially what rosmake does)
      # but I'm not sure its really saving any time over just
      # building all of the catkin packages
	catkinBuild $team_repo_dir true ${rosbuild_pkgs[@]} ${catkin_pkgs[@]}|| return $?

	rosbuildWithDeps $team_repo_dir ${rosbuild_pkgs[@]} || return $?
    else
      if [ $i_catkin -gt 0 ]; then
        catkinBuild $team_repo_dir true ${catkin_pkgs[@]} || return $?
      fi
    fi

  else
    echo "Making the following packages"

    i_catkin=0 catkin_pkgs=()
    i_rosbuild=0 rosbuild_pkgs=()    
    for pkg in $@; do
      echo $pkg
        if $(isCatkinPackage $team_repo_dir $pkg); then
          catkin_pkgs[$i_catkin]=$pkg
          ((++i_catkin))
        else
          rosbuild_pkgs[$i_rosbuild]=$pkg
          ((++i_rosbuild))
        fi
    done

    if [ $i_catkin -gt 0 ]; then
      catkinBuild $team_repo_dir false ${catkin_pkgs[@]} || return $?
    fi

    if [ $i_rosbuild -gt 0 ]; then
      rosbuild $team_repo_dir false ${rosbuild_pkgs[@]} || return $?
    fi
  fi
fi



