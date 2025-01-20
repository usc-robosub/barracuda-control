FROM ros:noetic-ros-base-focal

RUN sudo apt-get update \
    && sudo apt-get install -y git vim wget \
    ros-noetic-tf \
    ros-noetic-tf2 \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/barracuda-control

# Set working directory
WORKDIR /opt

RUN . /opt/ros/noetic/setup.sh \ 
    && cd /opt/barracuda-control/catkin_ws \
    && catkin build

# Source the workspace on container start
CMD ["/bin/bash", "/opt/barracuda-control/entrypoint.sh"]