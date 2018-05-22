

# Invite-Robotics
This repository contains the integration of the [CSDA10F dual-arm Motoman Robot](https://www.yaskawa.eu.com/uk/products/robotic/motoman-robots/productdetail/product/csda10f/) with two [2-figer adaptive Robotiq grippers](https://robotiq.com/support/2-finger-adaptive-robot-gripper) and custom sensors with ROS framework. Aiming to develop robotics applications at Invite GmbH Research Center.


![all](https://user-images.githubusercontent.com/8356912/38436175-7e44e2ac-3999-11e8-8157-80c98658412e.jpg)

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
* [Release notes](https://github.com/Danfoa/invite-robotics/wiki/release-notes)
* [Notes for safety operation](https://github.com/Danfoa/invite-robotics/wiki/Notes-for-safety-operation)
***
## About The Project
This is a non-profit/collaborative project between the National University of Colombia and Invite GmbH research center, that intends to allow Invite to develope more complex/cooler task without them facing alone the already challenging ROS learning curve. In order to achieve this we are trying to set-up most required robot configurations and write/migrate examples and tutorials of common task with the CSDA10F robot. 

## Project Facts/Datasheet
| Function                            |    Status       | 
| :---                                |     :---:       |       
| MoveIt! Configuration               |  :ok_hand:      |
| CSDA10F support package             |  :ok_hand:      | 
| Configuration files for real robot operation                |  :ok_hand:      | 
| MoveGroupInterface tutorial with CSDA10F               |  :ok_hand:      |
| TRAC-IK Integration for most move groups               |  :ok_hand:      | 
| Motion planning with 15 DoF            |  :ok_hand:   |  
| Gazebo robot simulation             |  :ok_hand:      | 
| Gazebo grippers simulation          |  :cry:          | 
| Grippers control through ROS control|  :neutral_face: | 

:ok_hand:: working/supported

:neutral_face:: almost there

:cry:: haven't addressed this issue