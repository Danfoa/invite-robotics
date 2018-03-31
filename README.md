# Invite-Robotics
This repository contains the integration of the CSDA10F dual-arm Motoman Robot with two Robotiq grippers and custom sensors with ROS framework. Aiming to develop robotics applications at Invite GmbH Research Center.

For further information please see the repository [Wiki](https://github.com/Danfoa/invite-robotics/wiki).

<img src="https://user-images.githubusercontent.com/8356912/37572017-fa8b24be-2b04-11e8-8f25-c2ea9d584550.png" width="600">

 _Dual arm motion planning with the configured CSDA10F robot URDF and Moveit configuration package_

## Documentation
* [Installation Instructions](https://github.com/Danfoa/invite-robotics/wiki/Intallation)
* [CSDA10F Robot Moveit configuration](https://github.com/Danfoa/invite-robotics/wiki/CSDA10F-Moveit-Configuration)
* Tutorials
   * [Move Group Interface Tutorial Cpp](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Move-Group-Interface-Cpp) 
   * [Modify Robot Grippers](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Modify-Robot-Grippers)
    * [Simulation Scene Objects](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Simulation-Scene-Objects)

## Version Changes
Please note for dependency changes with every stable (betha) release. 
### v0.1.3
* New inverse kinematics solver ([Track_IK](https://bitbucket.org/traclabs/trac_ik)) implemented for `arm_left` and `arm_right` move groups; this solver impoves heavily the planning behaviour near singulatiry points and joint limits, plus a general increase in robustness and speed - new dependency added: `trac_ik_kinematics_plugin` (to package `invite_motoman_moveit_config`)
* New tutorial added: [Simulation Scene Objects](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Simulation-Scene-Objects)
* New tutorial added: [Modify Robot Grippers](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Modify-Robot-Grippers)
***
***
## About The Project
This is a non-profit/collaborative project between the National University of Colombia and Invite GmbH research center, that intends to allow Invite to develope more complex/cooler task without them facing alone the already challenging ROS learning curve. In order to achieve this we are trying to set-up most required robot configurations and write/migrate examples and tutorials of common task with the CSDA10F robot. 

![unal invite](https://user-images.githubusercontent.com/8356912/38167608-0c92588e-3538-11e8-8ea0-4f41dd4b69e1.png)
