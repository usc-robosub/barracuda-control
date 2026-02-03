FROM ros:humble-ros-base-jammy

# Install build and ROS dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       git vim wget \
       libeigen3-dev libboost-all-dev liblapack-dev \
       python3-colcon-common-extensions doxygen \
       ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
       ros-humble-geometry-msgs ros-humble-nav-msgs \
       ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/barracuda-control

# Set working directory
WORKDIR /opt


RUN . /opt/ros/humble/setup.sh && \
    cd /opt/barracuda-control/dependencies/blasfeo && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DBLASFEO_EXAMPLES=OFF \
      -DTARGET=GENERIC && \
    make -j4 && make install -j && \
    ldconfig && \
    cd ../../hpipm && mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DHPIPM_TESTING=OFF \
      -DTARGET=GENERIC && \
    make -j4 && make install -j && \
    ldconfig && \
    cd /opt/barracuda-control/catkin_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select thruster_manager

# Source the workspace on container start
CMD ["/bin/bash", "/opt/barracuda-control/entrypoint.sh"]
