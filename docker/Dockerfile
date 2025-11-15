FROM ros:humble-perception-jammy

ENV DEBIAN_FRONTEND=noninteractive

# -------------------------------------------------------
# APT 依赖（ROS 工具 + RViz2 所需图形库 + 额外包）
# -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Python 包管理工具
    python3-pip \
    # ROS2 可视化和状态发布工具
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-pinocchio \
    # RViz2 GUI 相关依赖
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libx11-xcb1 \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------
# PIP 依赖
# -------------------------------------------------------
# -------------------------------
# 配置 pip 国内源加速
# -------------------------------
# RUN pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

RUN pip install --no-cache-dir xacro hydra-core termcolor colorama dynamixel-sdk zmq
RUN pip install --no-cache-dir dex_retargeting
RUN pip install --no-cache-dir opencv-python
RUN pip install --no-cache-dir mediapipe==0.10.14
RUN pip install --no-cache-dir numpy==1.26.4 # 解决 mediapipe 与 numpy 最新版本冲突的问题

# -------------------------------------------------------
# 创建标准 ROS2 工作空间
# -------------------------------------------------------
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# -------------------------------------------------------
# 默认启动 bash（不会自动 colcon build）
# -------------------------------------------------------
CMD ["bash"]
