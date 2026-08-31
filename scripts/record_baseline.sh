#!/usr/bin/env bash

set -euo pipefail

OUT="$HOME/go2_mid360_nav/datasets/baseline"

mkdir -p "$OUT"

STAMP="$(date +%Y%m%d_%H%M%S)"

rosbag record \
  -O "$OUT/go2_edu_02_baseline_${STAMP}.bag" \
  /livox/lidar \
  /livox/imu \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/depth/image_rect_raw \
  /camera/depth/camera_info \
  /tf \
  /tf_static \
  /Odometry \
  /odom_nav
