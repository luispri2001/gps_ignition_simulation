# GPS Navigation for Ignition Fortress

[![Code Size](https://img.shields.io/github/languages/code-size/luispri2001/gps_ignition_simulation.svg)](https://github.com/luispri2001/gps_ignition_simulation) [![Last Commit](https://img.shields.io/github/last-commit/luispri2001/gps_ignition_simulation.svg)](https://github.com/luispri2001/gps_ignition_simulation/commits/main) [![GitHub issues](https://img.shields.io/github/issues/luispri2001/gps_ignition_simulation)](https://github.com/luispri2001/gps_ignition_simulation/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/luispri2001/gps_ignition_simulation)](https://github.com/luispri2001/gps_ignition_simulation/pulls) [![Contributors](https://img.shields.io/github/contributors/luispri2001/gps_ignition_simulation.svg)](https://github.com/luispri2001/gps_ignition_simulation/graphs/contributors)

## Tested Systems and ROS 2 Distro
| System        | ROS 2 Distro | Build Status |
|---------------|--------------|--------------|
| Ubuntu 22.04  | Humble       | ![Build Status](https://github.com/luispri2001/gps_ignition_simulation/actions/workflows/main.yml/badge.svg?branch=simulation) |

This repository is a based on the original [navigation2_tutorials](https://github.com/ros-planning/navigation2_tutorials) repository. It has been extensively modified to support migration to **Ignition Fortress**. In addition to migrating the TurtleBot simulation, new robots such as the **Leo Rover** have been added, along with new maps—primarily of the University of Leon campus.

## Changes and Improvements
- Adaptation of code for compatibility with ROS 2 Humble and migration to Ignition Fortress.
- Migration of the TurtleBot simulation to Ignition Fortress.
- Addition of new robot models, including the Leo Rover.
- Inclusion of new maps, focusing on the campus of the University of Leon.

## Usage
To clone this repository and compile it within a ROS 2 Humble workspace:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/luispri2001/gps_ignition_simulation.git
cd ~/ros2_ws
colcon build --symlink-install
```

---

## Funding

This work has been funded under the following research projects:

### SELF-AIR Project

Supporting Extensive Livestock Farming with the use of Autonomous Intelligent Robots

<img src="https://raw.githubusercontent.com/shepherd-robot/.github/main/profile/robotics_wolf_minimal.png" alt="SELF_AIR_logo" width="50%" height="50%">

Grant TED2021-132356B-I00 funded by MCIN/AEI/10.13039/501100011033 and by the “European Union NextGenerationEU/PRTR”

![SELF_AIR_EU eu_logo](https://raw.githubusercontent.com/shepherd-robot/.github/main/profile/micin-financiadoUEnextgeneration-prtr-aei.png)
