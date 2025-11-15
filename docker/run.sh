#!/bin/bash
set -e

# 自动切换到 run.sh 所在目录的上一级（项目根目录）
cd "$(dirname "$0")/.."

# 允许 root 访问 X11（RViz2 所需）
xhost +si:localuser:root

docker run -dit \
  --name mocam_container \
  --privileged \
  -v /dev:/dev \
  -v "$(pwd):/root/ros2_ws" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -w /root/ros2_ws \
  --net=host \
  mocam_image
