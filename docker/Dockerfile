FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    vim \
    i2c-tools \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

WORKDIR /ros2_ws

COPY barracuda_control /ros2_ws/src/barracuda_control
COPY thruster_manager /ros2_ws/src/thruster_manager

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> ~/.bashrc \
    && sed -i '1iforce_color_prompt=yes' ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]

# RUN apt-get update && apt-get install -y \
#     git \
#     openssh-client \
#     vim \
#     # ros-humble-foxglove-bridge \
#     # ros-humble-xacro \
#     # ros-humble-joint-state-publisher-gui \

#     && rm -rf /var/lib/apt/lists/*

# COPY . /opt/barracuda-control

# RUN . /opt/ros/humble/setup.sh && \
#     cd /opt/barracuda-control/dev_ws && \
#     colcon build --symlink-install

# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
#     && echo "[ -f /opt/barracuda-control/dev_ws/install/setup.bash ] && source /opt/barracuda-control/dev_ws/install/setup.bash" >> ~/.bashrc \
#     && sed -i '1iforce_color_prompt=yes' ~/.bashrc



# WORKDIR /opt/barracuda-control/dev_ws
# CMD ["/bin/bash", "/opt/barracuda-control/entrypoint.sh"]
