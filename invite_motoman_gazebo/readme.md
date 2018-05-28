# invite_motoman_Gazebo

##Overview

This package contains the files required to simulate the motoman CSDA 10F dual arm with grippers 
Robotiq in Gazebo. 


## Using Moveit! with Gazebo Simulator

1. Bring the robot model into gazebo and load the ros_control controllers:
     
   ``` roslaunch invite_motoman_gazebo invite_motoman_gazebo.launch ```

   -	If you want load the scene of invite laboratory, you should pass as argument (scene:=true)  
	and waiting a several seconds while the all elements are loaded. 

   - 	After the simulator is running, continue with te next step.
	

2. Launch moveit! and ensure that it is configured to run alongside Gazebo:
        
    ```roslaunch invite_motoman_moveit_config moveit_planning_execution_invite_gazebo.launch``` 
