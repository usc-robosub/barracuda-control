FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    vim \
    ros-humble-foxglove-bridge \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \

    && rm -rf /var/lib/apt/lists/*

COPY . /opt/barracuda-control

RUN . /opt/ros/humble/setup.sh && \
    cd /opt/barracuda-control/dev_ws && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f /opt/barracuda-control/dev_ws/install/setup.bash ] && source /opt/barracuda-control/dev_ws/install/setup.bash" >> ~/.bashrc \
    && sed -i '1iforce_color_prompt=yes' ~/.bashrc



WORKDIR /opt/barracuda-control/dev_ws
CMD ["/bin/bash", "/opt/barracuda-control/entrypoint.sh"]
