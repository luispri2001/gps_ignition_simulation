# **GPS Navigation for Ignition Fortress and Gazebo Classic**  

[![Code Size](https://img.shields.io/github/languages/code-size/luispri2001/gps_ignition_simulation.svg)](https://github.com/luispri2001/gps_ignition_simulation) [![Last Commit](https://img.shields.io/github/last-commit/luispri2001/gps_ignition_simulation.svg)](https://github.com/luispri2001/gps_ignition_simulation/commits/main) [![GitHub issues](https://img.shields.io/github/issues/luispri2001/gps_ignition_simulation)](https://github.com/luispri2001/gps_ignition_simulation/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/luispri2001/gps_ignition_simulation)](https://github.com/luispri2001/gps_ignition_simulation/pulls) [![Contributors](https://img.shields.io/github/contributors/luispri2001/gps_ignition_simulation.svg)](https://github.com/luispri2001/gps_ignition_simulation/graphs/contributors)  

## **Tested Systems and ROS 2 Distro**  
| System        | ROS 2 Distro | Ignition Fortress | Gazebo Classic | Build Status |
|--------------|-------------|-------------------|---------------|--------------|
| Ubuntu 22.04 | Humble      | ✅               | ✅            | ![Build Status](https://github.com/luispri2001/gps_ignition_simulation/actions/workflows/main.yml/badge.svg?branch=main) |  

This repository is based on the original [navigation2_tutorials](https://github.com/ros-planning/navigation2_tutorials) repository. It has been extensively modified to support migration to **Ignition Fortress**. In addition to migrating the TurtleBot simulation, new robots such as the **Leo Rover** and **Unitree Go2** have been added, along with new maps—primarily of the University of Leon campus.  

## **Changes and Improvements**  
- Adaptation of code for compatibility with ROS 2 Humble and migration to Ignition Fortress.  
- Migration of the TurtleBot simulation to Ignition Fortress.  
- Addition of new robot models, including the Leo Rover and Unitree Go2.  
- Inclusion of new maps, focusing on the campus of the University of Leon.

## **Usage**  
To clone this repository and compile it within a ROS 2 Humble workspace:  

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/luispri2001/gps_ignition_simulation.git
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## **Map Visualization with Mapviz**  
For visualizing maps in **Mapviz**, we use a **Docker container** based on [danielsnider/docker-mapproxy-googlemaps](https://github.com/danielsnider/docker-mapproxy-googlemaps).  

### **Setup and Launch**  
To start the **Mapproxy** Docker container, run:  

```sh
docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```

### **How It Works**  
- This container fetches **Google Maps tiles** and serves them locally.  
- Mapviz can then access these tiles from `http://localhost:8080`.  
- The `-v ~/mapproxy:/mapproxy` option ensures that cached tiles are stored persistently in `~/mapproxy`.  

## **Launching the Simulation**  

### **Leo Rover in Ignition Fortress**  
To launch the simulation with the **Leo Rover** and visualize it in **RViz** and **Mapviz**, use the following command:  

```sh
ros2 launch nav2_gps_waypoint_follower_demo leo_ign_gps_waypoint_follower.launch.py use_rviz:=true use_mapviz:=true
```

### **Unitree Go2 in Gazebo Classic**  
To launch the **Unitree Go2** simulation in **Gazebo Classic** with GPS support, use:  

```sh
ros2 launch nav2_gps_waypoint_follower_demo go2_gazebo_gps_waypoint_follower.launch.py use_rviz:=true use_mapviz:=true
```

## **Moving to Clicked Points on the Map**  
To move the robots to points that are clicked on the map, launch the following node:  

```sh
ros2 run nav2_gps_waypoint_follower_demo interactive_waypoint_follower
```

## **Funding**  

This work has been funded under the following research projects:  

### **SELF-AIR Project**  
Supporting Extensive Livestock Farming with the use of Autonomous Intelligent Robots  

<img src="https://raw.githubusercontent.com/shepherd-robot/.github/main/profile/robotics_wolf_minimal.png" alt="SELF_AIR_logo" width="50%" height="50%">

Grant TED2021-132356B-I00 funded by MCIN/AEI/10.13039/501100011033 and by the “European Union NextGenerationEU/PRTR”  

![SELF_AIR_EU eu_logo](https://raw.githubusercontent.com/shepherd-robot/.github/main/profile/micin-financiadoUEnextgeneration-prtr-aei.png)  

