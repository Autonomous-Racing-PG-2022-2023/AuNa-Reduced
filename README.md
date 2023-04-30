# Autonomous Navigation System Simulator
___
This repository includes a complete ROS2 package for the simulation of autonomous robots. It features the simulation models, navigation algorithms and other components to run and evaluate cooperative driving scenarios. Each scenario can be extended to feature different robots, additional system components and more. The launch files are modularly built, so that each part can be configured without directly affecting the other components of the simulation.

Additionally, it integrates ROS2-Foxy with MATLAB/Simulink and OMNeT++/Artery, to enable the integration of control systems and communication standards. Currently, it includes a CACC-controller for platooning and an implementation of the ETSI-ITS-G5 communication architecture.

![](https://github.com/HarunTeper/AuNa/blob/main/media/gazeboSimulation.gif)

## Package Setup and Overview
___
### Installation

The following steps explain the required installation steps to run the framework on a machine running Ubuntu 20.04:

First, install the following packages and tools as described here:

    https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
    http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
    
Then, additionally install the following packages:
    
    sudo apt install python3-pip
    sudo apt install python3-colcon-common-extensions
    sudo apt install ros-galactic-xacro
    sudo apt install ros-galactic-rmw-cyclonedds-cpp
    sudo apt install ros-galactic-gazebo-ros-pkgs
    sudo apt install ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-turtlebot3 ros-galactic-turtlebot3-*
    pip install ruamel.yaml

After that, build the package:

    source /opt/ros/galactic/setup.bash
    colcon build --symlink-install
        
Run the following commands in the terminal before using ROS2:

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export GAZEBO_MODEL_PATH=~/AuNa/src/car_simulator/models:$GAZEBO_MODEL_PATH
    
    export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org/
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/galactic/share/turtlebot3_gazebo/models
    
    source /opt/ros/galactic/setup.bash
    source ~/AuNa-Reduced/install/setup.bash

### File stucture:
```
├── car_simulator
│   ├── car_simulator
│   │   └── yaml_launch.py  #Includes commands to read and configure .yaml files
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── map_params #Parameter files for maps, such as spawn locations and others
│   │   ├── model_params # Model paramters of each robot model
│   │   ├── nav2_params # Navigation parameters for Nav2 nodes
│   │   └── scenario_params # Scenario parameters for robot nodes
│   ├── include # Include files for scripts in src
│   ├── launch # Launch files
│   │   ├── gazebo # Gazebo launch files for arbitrary world files
│   │   ├── navigation # Navigation launch files for single and multiple robots
│   │   ├── omnet # OMNeT++ launch files to launch bridge-nodes for communication with OMNeT++ and Artery
│   │   ├── scenarios # Currently implemented launch files for custom scenarios
│   │   └── spawn # Launch files to correctly spawn robots in Gazebo
│   ├── maps # Map files for Gazebo worlds
│   │   └── racetrack_decorated
│   ├── matlab
│   │   ├── CACC # Platooning controller implementation in MATLAB and Simulink
│   ├── models # Implemented robot models and world model files
│   │   ├── prius_custom
│   │   ├── race_car
│   │   └── racetrack
│   ├── package.xml
│   ├── rviz # RViz configuration files for scenarios
│   ├── scripts # Python scripts for scenarios and tools
│   │   ├── teleoperation # Scripts for keyboard controls
│   ├── src # C++ scripts
│   │   ├── omnet # OMNeT++ bridge-nodes
│   │   └── transformations # Transformation nodes for multi-robot setups
│   └── worlds # World files for Gazebo
├── etsi_its_msgs # ETSI-ITS-G5 messages for OMNeT++ and Artery
└── ros_its_msgs # CAM simple message format
```
	
## How to use?
___
## ROS2

After building the package, the currently implemented scenarios can be found in */src/car_simulator/launch/scenarios*. The multi-robot navigation scenario can be launched as follows:

    ros2 launch car_simulator scenario_robot_racetrack.launch.py robot_number:=4
    
Make the car drive using:
    
    ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.1}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    
## Acknowledgements

We would like to thank all the authors who helped to extend the framework. In particular, we would like to thank Anggera Bayuwindra, Enio Prates Vasconcelos Filho, Raphael Riebl, and Ricardo Severino for providing their components and implementation details for the integration.
