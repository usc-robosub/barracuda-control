FROM ros:humble-ros-base-jammy

WORKDIR /ros2_ws
COPY barracuda_control /ros2_ws/src/barracuda_control
COPY thruster_manager /ros2_ws/src/thruster_manager

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN rosdep update && \
    apt-get update && \
    apt-get install -y python3-pip && \
    rosdep install --from-paths src --ignore-src -y

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc \
    && sed -i '1iforce_color_prompt=yes' ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
