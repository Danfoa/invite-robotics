

# Invite-Robotics
This repository contains the integration of the [CSDA10F dual-arm Motoman Robot](https://www.yaskawa.eu.com/uk/products/robotic/motoman-robots/productdetail/product/csda10f/) with two [2-figer adaptive Robotiq grippers](https://robotiq.com/support/2-finger-adaptive-robot-gripper) and custom sensors with ROS framework. Aiming to develop robotics applications at Invite GmbH Research Center.

![cover2](https://user-images.githubusercontent.com/8356912/40397357-ef27e30c-5df7-11e8-92bd-06e8669fd6d0.jpg)


For further information please see the repository [Wiki](https://github.com/Danfoa/invite-robotics/wiki).
## Contents
* [Installation Instructions](https://github.com/Danfoa/invite-robotics/wiki/Intallation)
* [CSDA10F Robot Moveit configuration](https://github.com/Danfoa/invite-robotics/wiki/CSDA10F-Moveit-Configuration)
* Tutorials
    * [Move Group Interface Tutorial Cpp](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Move-Group-Interface-Cpp) 
    * [Connect to Robot Controller](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Connect-to-Robot-Controller)
    * [Pick and Place Test](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Robot-Test-Pick-and-Place)  
    * [Connect to Robotiq gripper through Modbus RTU](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Connect-to-Robotiq-grippers-through-ModbusRTU) 
    * [Modify Robot Grippers](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Modify-Robot-Grippers)
    * [Simulation Scene Objects](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Simulation-Scene-Objects) 
    * [3D Sensor Integration](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---3D-Sensor-Integration)   
* [Release notes](https://github.com/Danfoa/invite-robotics/wiki/release-notes)
* [Notes for safety operation](https://github.com/Danfoa/invite-robotics/wiki/Notes-for-safety-operation)
***
## About The Project
This is a non-profit/collaborative project between the National University of Colombia and Invite GmbH research center, that intends to allow Invite to develope more complex/cooler task without them facing alone the already challenging ROS learning curve. In order to achieve this we are trying to set-up most required robot configurations and write/migrate examples and tutorials of common task with the CSDA10F robot. 

## Project Facts/Datasheet
| Function                            |    Status       | 
| :---                                |     :---:       |       
| MoveIt! Configuration               |  Supported      |
| CSDA10F support package             |  Supported     | 
| Configuration files for real robot operation                |  Supported      | 
| MoveGroupInterface tutorial with CSDA10F               |  Supported     |
| TRAC-IK Integration for most move groups               |  Supported      | 
| Motion planning with 15 DoF            |  Supported   | 
| Octomap from realsense RS200          |  Supported   |   
| Gazebo robot simulation             |  Supported      | 
| Gazebo grippers simulation          |  In progress          | 
| Grippers control through ROS control|  In progress | 
