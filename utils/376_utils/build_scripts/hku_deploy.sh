#!/bin/bash
set -e # stop on error

echo "Deploying to ocs4, ocs2, fc1..."

time rsync -av -e ssh --exclude='.git' --exclude='build' --delete ~/ros_workspace ocs@ocs2:
time rsync -av -e ssh --exclude='.git' --exclude='default_warehouse_mongo_db' --exclude='build' --delete ~/ros_workspace fc@fc1:

echo "Done!"

