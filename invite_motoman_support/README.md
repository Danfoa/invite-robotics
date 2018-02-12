## Description
This package sets up the .urdf robot description of Invite's YASKAWA CSDA10F dual arm robot, its grippers (2-fingers Adaptative robotiq Grippers) and Wacoh-Tech Dynpick force/torque sensor.
![invite_motoman_support](https://user-images.githubusercontent.com/8356912/36078334-fe103bde-0f74-11e8-896b-b90492269947.png)

## How-to
For modifying robot and grippers configuration the .xacro files under 'urdf' folder need to be changed.

In order to see the robot and/or grippers configuration on Rviz, several launch files under the 'launch' folder are already configured for the task.

For example to see the entire robot configuration type on the console:
  ```
  roslaunch invite_motoman_support test_csda10f.launch
  ```
To see the right arm gripper configuration type on the console:
  ```
  roslaunch invite_motoman_support test_right_gripper.launch
  ```
## To Do
- [ ] Better visual CAD files for the Force sensor and Force sensor adapter plate.
- [ ] Place fingertips on right arm gripper 
- [ ] Search for a more "decent" .xacro for the left arm gripper. 
