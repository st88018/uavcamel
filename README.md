# UAVCAMEL
UAV Create Any Movement Easy Look

## Packages
This repository provide a easy and safe way to command the UAV in the VICON room.

## Basic usage of the ROS packages

1. Install dependent libraries

    ```
    sudo apt install -y ros-$ROS_DISTRO-mavros
    sudo apt install -y ros-$ROS_DISTRO-vrpn
    sudo apt install -y ros-$ROS_DISTRO-vrpn-client-ros
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/camel_ws/src)

    ```
    mkdir -p ~/camel_ws/src
    cd ~/scout_ws/src
    git clone https://github.com/st88018/uavcamel.git  
    cd ..
    catkin_make
    ```