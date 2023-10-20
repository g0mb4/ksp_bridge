FROM osrf/ros:humble-desktop

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

RUN ldconfig

### Bridge into it 
##  Build ksp_bridge
RUN mkdir -p /ros2_ws
WORKDIR /ros2_ws
RUN git clone https://github.com/clausqr/ksp_bridge.git src
RUN . /opt/ros/humble/setup.sh && \
      colcon build 
RUN echo "source /ros2_ws/install/local_setup.bash" >> /root/.bashrc

##  Build ksp_bridge
# $ docker build -t clausqr:ksp_bridge . 
## Running this container
# $ docker run -it -v $(pwd):/ros_ws/src --net=host clausqr:ksp_bridge
## Inside:
# $ ros2 launch ksp_bridge_examples resource_monitor.launch.py
