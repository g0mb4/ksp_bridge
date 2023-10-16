FROM ros:humble

# Install dependencies
RUN apt update && apt install -y libprotobuf-dev libprotoc-dev protobuf-compiler wget unzip sudo libasio-dev


# Install kRPC
RUN mkdir krpc && \
    cd krpc && \
    wget https://github.com/krpc/krpc/releases/download/v0.5.2/krpc-cpp-0.5.2.zip && \ 
    unzip krpc-cpp-0.5.2.zip && \
    cd krpc-cpp-0.5.2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install

### Bridge into it 
##  Build ksp_bridge
# RUN mkdir -p /ros2_ws/src
# WORKDIR /ros2_ws/src
# RUN git clone https://github.com/clausqr/ksp_bridge.git
# WORKDIR /ros2_ws
# RUN . /opt/ros/humble/setup.sh && \
#     colcon build --symlink-install