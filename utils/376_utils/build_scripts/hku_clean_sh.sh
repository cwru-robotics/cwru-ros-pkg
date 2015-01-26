#!/usr/bin/env bash

# get the notify function
. ./build_functions.sh

getTeamRepo || return $?

if [ -z $1 ]; then
  catkinClean $team_repo_dir || return $?
  rosbuildClean $team_repo_dir || return $?
else
  if $(isCatkinPackage $team_repo_dir $1); then
    catkinClean $team_repo_dir $1 || return $?
  else
    rosbuildClean $team_repo_dir $1 || return $?
  fi
fi
