#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo " Barracuda Control Workspace Ready! "
echo "=========================================="

exec "$@"