# invite_motoman_Gazebo

##Overview

This package contains the files required to simulate the motoman CSDA 10F dual arm with grippers 
Robotiq in Gazebo. 




## Using Moveit! with Gazebo Simulator

1. Bring the robot model into gazebo and load the ros_control controllers:
   ```roslaunch `rospack find invite_motoman_gazebo`/launch/invite_motoman_gazebo.launch``` 
   
   ```roslaunch invite_motoman_gazebo.launch```

2. Launch moveit! and ensure that it is configured to run alongside Gazebo:
    ```roslaunch `rospack find invite_motoman_moveit_config`/launch/moveit_planning_execution_invite_gazebo.launch``` 
    
    ```roslaunch moveit_planning_execution_invite_gazebo.launch``` 
