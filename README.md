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
3. Source the workspace
   ```
   source ~/camel_ws/devel/setup.bash
   ```
## Play UAVCAMEL in PX4 Software Simulation
1. Downloadn and install the PX4 (1.11.0)
   ```
   git clone https://github.com/PX4/PX4-Autopilot.git
   cd PX4-Autopilot/
   git checkout 71db090
   git submodule sync --recursive
   git submodule update --init --recursive
   bash ./Tools/setup/ubuntu.sh
   sudo apt upgrade libignition-math2
   ```
2. Launch the Simulation environment
    (Gazebo)
    ```
    make px4_sitl_default gazebo
    ```
    (Jmavsim)
    ```
    make px4_sitl_default jmavsim
    ```
2. Launch Mavros
    ```
    source ~/camel_ws/devel/setup.bash
    roslaunch uavcamel px4_sitl.launch
    ```
2. RUN UAVCAMEL
    ```
    rosrun uavcamel camel
    ```
## UAVCAMEL Misson documentation

3. Please make sure the path is correct at cammel.cpp line 54
    ```
    string MissionPath = "/home/usr/camel_ws/src/uavcamel/src/utils/Missions/Mission.csv";
    ```
2. CSV file
    The [Mission.csv](src/utils/Missions/Mission.csv) is the preprogrammed mission file:

|  Mission Type |               |               |               |               |               |               |               |
| ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
| 1.TakeOff     |               |               |   Z position  |               |               |               |  wait time(s) |
| 2.constVtraj  |   X position  |   Y position  |   Z position  |  heading(rad)  |   velocity    |  angular vel  |  wait time(s) |
| 3.AMtraj      |   X position  |   Y position  |   Z position  |  heading(rad)  |               |               |               |
| 4.RTL         |               |               |               |  heading(rad)  |   velocity    |  angular vel  |  wait time(s) |
| 5.Land        |               |               |               |  heading(rad)  |   velocity    |  angular vel  |               |
