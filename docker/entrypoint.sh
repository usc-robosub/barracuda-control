#!/bin/bash
set -e

echo "Installing dependencies..."
apt-get update

rosdep install --from-paths src --ignore-src -y

echo "Building ROS 2 workspace..."
# colcon build --symlink-install

# source install/setup.bash

echo "=========================================="
echo " Barracuda Control Workspace Ready! "
echo "=========================================="

exec "$@"