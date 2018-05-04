# Invite's Motoman CSDA10F Moveit Configuration

The [invite_motoman_moveit_config](https://github.com/Danfoa/invite-robotics/tree/stable/invite_motoman_moveit_config) package host the MoveIt! configuration of the Motoman CSDA10F with robotiq (85mm and 140mm stroke) [2-finger adaptive grippers](https://robotiq.com/support/2-finger-adaptive-robot-gripper).

## Move Groups

The following move groups were configured:
* `csda10f`: Move group of all actuated joints in the CSDA10F robot.
* `arms`:  Move group of both arms without the torso joint.
* `arm_left`: Move group of all the joints in the left arm with the 85mm Robotiq gripper.
* `arm_right`: Move group of all the joints in the right arm with the 140mm Robotiq gripper.
* `torso`: Move group with only the torso as actuated joint.

In order to see the move groups in link level detail, launch the Moveit setup assistant and go through the configuration.
```
roslaunch invite_motoman_moveit_config setup_assistant.launch
```
<img src="https://user-images.githubusercontent.com/8356912/37992252-781c3a0a-320b-11e8-8fe2-1dfbe5ac03e1.png" width="600"> _csda10f_
<img src="https://user-images.githubusercontent.com/8356912/37992251-77f82dfe-320b-11e8-813c-f704a0a404e5.png" width="600"> _arms_
<img src="https://user-images.githubusercontent.com/8356912/37992257-7c36607a-320b-11e8-94c9-e9fb7faf8b52.png" width="600"> _arm_left_
<img src="https://user-images.githubusercontent.com/8356912/37992259-7c7d79d8-320b-11e8-8523-958b5e4bf910.png" width="600"> _arm_right_


## End effectors
Both robotiq grippers are already configured as end effectors with TCP in the `arm_left_link_tcp` and `arm_right_link_tcp` links.

<img src="https://user-images.githubusercontent.com/8356912/37992258-7c5dc7dc-320b-11e8-8a8a-cf8d08433623.png" width="600"> _left_gripper_
<img src="https://user-images.githubusercontent.com/8356912/37992260-7c9ed3bc-320b-11e8-9aa6-6f7705d7dcc0.png" width="600"> _right_gripper_

## Kinematic Solver

* `csda10f`:  _KDL_kinematics_plugin_
* `arms`:   _KDL_kinematics_plugin_
* `arm_left`: _trac_ik_kinematics_plugin_
* `arm_right`: _trac_ik_kinematics_plugin_
* `torso`:  _KDL_kinematics_plugin_

 [Track_IK](https://bitbucket.org/traclabs/trac_ik) is selected since it has proven its superiority over KDL over a different set of robots (none of them Motoman), specifically on 7 DoF arms used in two arm robots (i.g. Valkyre, Pr2, Baxter, Robonaut). Currently it doesnt support mimic joints which makes it impossible to integrate over any moove group that uses the torsoo joint. 

 <img src="https://user-images.githubusercontent.com/8356912/38168982-5c523022-355d-11e8-96eb-1c79a1f6dd19.png" width="800">
 
_Comparison between KDL and Track IK kineamics solvers_
