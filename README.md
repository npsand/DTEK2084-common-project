# `drone_racing_ros2`
## Running a Tello simulation in [Gazebo](http://gazebosim.org/)

`tello_gazebo` consists of several components:
* `TelloPlugin` simulates a drone, handling takeoff, landing and very simple flight dynamics
* `markers` contains Gazebo models for fiducial markers
* `fiducial.world` is a simple world with a bunch of fiducial markers
* `inject_entity.py` is a script that will read an URDF (ROS) or SDF (Gazebo) file and spawn a model in a running instance of Gazebo
* the built-in camera plugin is used to emulate the Gazebo forward-facing camera


## Installation
#### Install ROS2 Galactic
    https://docs.ros.org/ with the `ros-galactic-desktop` option.
#### Make sure you have gazebo 
    sudo apt install gazebo11 libgazebo11 libgazebo11-dev
#### Add the following
    sudo apt install libasio-dev
    sudo apt install ros-galactic-cv-bridge ros-galactic-camera-calibration-parsers 
    sudo apt install libignition-rendering3 
    pip3 install transformations

## Building
#### Build this package
    mkdir -p ~/drone_racing_ros2_ws/src
    cd ~/drone_racing_ros2_ws/src
    git clone https://github.com/npsand/DTEK2084-common-project
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build
    
## Running    
#### Run a teleop simulation

    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo simple_launch.py
    
You will see a single drone in a blank world.
You can control the drone using the joystick.

If you run into the **No namespace found** error re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    
#### Run a drone_racer controller

You must run simulation above before this!

    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    ros2 launch drone_racer drone_racer_simulation_launch.py
    
You can replace `drone_racer_simulation_launch.py` with `drone_racer_irl_launch.py` to run code with the real drone.

#### Control the drone 
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1






