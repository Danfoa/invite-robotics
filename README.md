

# Invite-Robotics
This repository contains the integration of the [CSDA10F dual-arm Motoman Robot](https://www.yaskawa.eu.com/uk/products/robotic/motoman-robots/productdetail/product/csda10f/) with two [2-figer adaptive Robotiq grippers](https://robotiq.com/support/2-finger-adaptive-robot-gripper) and custom sensors with ROS framework. Aiming to develop robotics applications at Invite GmbH Research Center.

## Contents
* [Installation Instructions](https://github.com/Danfoa/invite-robotics/wiki/Intallation)
* [Robot URDF configuration](https://github.com/Danfoa/invite-robotics/wiki/Robot-URDF-configuration)
* [Robot Moveit configuration](https://github.com/Danfoa/invite-robotics/wiki/CSDA10F-Moveit-Configuration)
* [Vision System Interface](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Vision-System-Interface)   
* Tutorials
  

  * [Connect to FS100 Robot Controller](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Connect-to-Robot-Controller)
  * [Move Group Interface Tutorial Cpp](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Move-Group-Interface-Cpp)
  * [CSDA10F Robot Interface](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---CSDA10F-Interface) 
  * [Cartesian Task Planner Cpp](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Cartesian-Task-Planning) 
  * [CSDA10F Pick and Place](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Robot-Test-Pick-and-Place)  
  * [Simulation Scene Objects](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Simulation-Scene-Objects)

* [Release notes](https://github.com/Danfoa/invite-robotics/wiki/release-notes)
* [Notes for safety operation](https://github.com/Danfoa/invite-robotics/wiki/Notes-for-safety-operation)
***

![new_home](https://user-images.githubusercontent.com/8356912/62937615-6d3a6680-bdcd-11e9-8d8a-c0b8e64db622.jpg)


For further information please see the repository [Wiki](https://github.com/Danfoa/invite-robotics/wiki).

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
| Motion execution of 15 DoF at a time            |  Supported   |    
| Gazebo robot simulation             |  Supported      | 
| Gazebo grippers simulation          |  In progress          | 
| Grippers control through ROS control|  Supported | 
