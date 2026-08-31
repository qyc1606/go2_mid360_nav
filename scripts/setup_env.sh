#!/usr/bin/env bash

set -u

source /opt/ros/noetic/setup.bash

if [ -d "/opt/unitree_robotics" ]; then
  export CMAKE_PREFIX_PATH="/opt/unitree_robotics:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="/opt/unitree_robotics/lib:${LD_LIBRARY_PATH:-}"
fi

if [ -d "/usr/local/lib" ]; then
  export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
fi

if [ -f "$HOME/go2_mid360_nav/catkin_ws/devel/setup.bash" ]; then
  source "$HOME/go2_mid360_nav/catkin_ws/devel/setup.bash"
fi

if [ -f "$HOME/go2_mid360_nav/vendor/ego-planner/devel/setup.bash" ]; then
  source "$HOME/go2_mid360_nav/vendor/ego-planner/devel/setup.bash"
fi
