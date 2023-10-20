# ksp_bridge

ROS2 package for Kerbal Space Program based on the kRPC mod.

Works with:  

+ KSP 1.12.5

+ kRPC 0.5.2

+ ROS2 Humble

+ Ubuntu 22.04

## Installation

### Build and install kRPC mod

Install the [kRPC mod](https://github.com/nullprofile/krpc) and it's dependencies.

``` bash
mkdir krpc
    cd krpc
    wget https://github.com/krpc/krpc/releases/download/v0.5.2/krpc-cpp-0.5.2.zip 
    unzip krpc-cpp-0.5.2.zip 
    cd krpc-cpp-0.5.2
    mkdir build
    cd build
    cmake ..
    make 
    make install
    ldconfig    # may need sudo!
```

### Build and install ksp_bridge

``` bash
cd ~/ros2_ws
git clone https://github.com/clausqr/ksp_bridge.git 
source /opt/ros/humble/setup.bash
colcon build 
```

## Usage

### 1. Server settings inside KSP

![up_and_down_assembly](doc/img/server_settings.jpg)

**Note**: *Max. time per update* is required to be high, but it affects the framerate.

### 2. Running ksp_bridge Examples

Resource Monitor

``` bash
source ~/ros2_ws/ksp_bridge/install/local_setup.bash
ros2 launch ksp_bridge_examples resource_monitor.launch.py
```

or

Up and Down Launch example:

``` bash
ros2 launch ksp_bridge_examples up_and_down.launch.py
```
