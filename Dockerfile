FROM ros:noetic-ros-base-focal

# Install build and ROS dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       git vim wget \
       libeigen3-dev libboost-all-dev liblapack-dev \
       python3-catkin-tools doxygen \
       ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
       ros-noetic-geometry-msgs ros-noetic-nav-msgs \
       ros-noetic-message-generation ros-noetic-message-runtime \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/barracuda-control

# Set working directory
WORKDIR /opt


RUN . /opt/ros/noetic/setup.sh && \
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
    catkin build -DCMAKE_BUILD_TYPE=Release

# Source the workspace on container start
CMD ["/bin/bash", "/opt/barracuda-control/entrypoint.sh"]
