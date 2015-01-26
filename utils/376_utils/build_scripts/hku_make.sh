#!/bin/bash

python ./hku_make.py "$@"
ret=$?

if [ "$JENKINS_BUILD" != "1" ]; then
  if [ -f /opt/hku_drc_team_scripts/setup.sh ]; then
    . /opt/hku_drc_team_scripts/setup.sh
  fi
fi

return $ret
