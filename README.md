# Invite-Robotics
Integration of the CSDA10F Motoman Robot with Robotiq grippers and custom sensors for developing of robotics applications at Invite Research Center.

## Installation
For instructions of installation and project setup see the [repository wiki](https://github.com/Danfoa/invite-robotics/wiki/Intallation)

<img src="https://user-images.githubusercontent.com/8356912/37572017-fa8b24be-2b04-11e8-8f25-c2ea9d584550.png" width="600">

 _Dual arm motion planning with the configured CSDA10F robot URDF and Moveit configuration package_
## Project Objectives
- [ ] Integration of the Motoman CSDA10F dual arm robot and FS100 controller with ROS framework
- [ ] Reduction of training time/ROS learning curve of Invite's emproyees by performing all the necesary robot/packages configuration and a series of tutorials and examples based directly on the CSDA10F.

## URDF Dependencies
### Yaskawa Motoman CSDA10F
The Universal Robot Description Files (URDF) of this robot were taken from the repository [motoman_csda10f_support](https://github.com/amrith1007/motoman_experimental/tree/kinetic-devel/motoman_csda10f_support).
### Robotiq 2-Finger Adaptive Grippers
The Robotiq URDF files were taken from:
- Gripper with 85mm stroke: [beta-bobots](https://github.com/beta-robots/robotiq)
- Gripper with 140mm stroke: [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq/tree/jade-devel/robotiq_arg2f_model_visualization)
