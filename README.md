# Invite-Robotics
This repository contains the integration of the CSDA10F dual-arm Motoman Robot with two Robotiq grippers and custom sensors with ROS framework. Aiming to develop robotics applications at Invite GmbH Research Center.

For further information please see the repository [Wiki](https://github.com/Danfoa/invite-robotics/wiki).

![all](https://user-images.githubusercontent.com/8356912/38436175-7e44e2ac-3999-11e8-8157-80c98658412e.jpg)

 _Dual arm motion planning with the configured CSDA10F robot URDF and Moveit configuration package_

## Documentation
* [Installation Instructions](https://github.com/Danfoa/invite-robotics/wiki/Intallation)
* [CSDA10F Robot Moveit configuration](https://github.com/Danfoa/invite-robotics/wiki/CSDA10F-Moveit-Configuration)
* Tutorials
   * [Move Group Interface Tutorial Cpp](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Move-Group-Interface-Cpp) 
   * [Modify Robot Grippers](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Modify-Robot-Grippers)
    * [Simulation Scene Objects](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Simulation-Scene-Objects)

## Release notes
### v0.1.5  - _Alpha release_
* Update to [MoveGroup Interface tutorial](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Move-Group-Interface-Cpp); New motion types included.
* Dependency updates to allow for easy rosdep commands, and easy installation
### v0.1.4  - _Alpha release_
* Dependency updates to allow for easy rosdep commands, and easy installation
### v0.1.3  - _Alpha release_
* New inverse kinematics solver ([Track_IK](https://bitbucket.org/traclabs/trac_ik)) implemented for `arm_left` and `arm_right` move groups; this solver impoves heavily the planning behaviour near singulatiry points and joint limits, plus a general increase in robustness and speed - new dependency added: `trac_ik_kinematics_plugin` (to package `invite_motoman_moveit_config`)
* New tutorial added: [Simulation Scene Objects](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Simulation-Scene-Objects); Default scene objects were pre configured for easy loading and using, see tutorial for instructions of use. 
* New tutorial added: [Modify Robot Grippers](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---Modify-Robot-Grippers); With this tutorial the grippers configuration was changed (i.g. the TCP was moved)
***
***
## About The Project
This is a non-profit/collaborative project between the National University of Colombia and Invite GmbH research center, that intends to allow Invite to develope more complex/cooler task without them facing alone the already challenging ROS learning curve. In order to achieve this we are trying to set-up most required robot configurations and write/migrate examples and tutorials of common task with the CSDA10F robot. 

![unal invite](https://user-images.githubusercontent.com/8356912/38168887-4cf70aec-355a-11e8-81ce-6c981da40067.png)