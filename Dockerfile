FROM ros:noetic-ros-base-focal

RUN sudo apt-get update \
    && sudo apt-get install -y git vim wget \
    ros-noetic-tf \
    ros-noetic-tf2 \
    libeigen3-dev \
    libboost-all-dev \    
    python3-catkin-tools \
    doxygen \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/barracuda-control

# Set working directory
WORKDIR /opt


RUN . /opt/ros/noetic/setup.sh \ 
    && cd /opt/barracuda-control/dependencies/blasfeo && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBLASFEO_EXAMPLES=OFF \
    && make -j4 \
    && sudo make install -j\ 
    && cd ../../hpipm && mkdir build && cd build\
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DHPIPM_TESTING=OFF \
    && make -j4 \
    && sudo make install -j\
    && export LD_LIBRARY_PATH=/opt/blasfeo/lib:/opt/hpipm/lib:$LD_LIBRARY_PATH \
    && cd /opt/barracuda-control/catkin_ws \
    && catkin build -DCMAKE_BUILD_TYPE=Release

# Source the workspace on container start
CMD ["/bin/bash", "/opt/barracuda-control/entrypoint.sh"]