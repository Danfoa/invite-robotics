cmake_minimum_required(VERSION 2.8.3)
project(invite_motoman_moveit_config)


find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
find_package(catkin REQUIRED
  trac_ik_kinematics_plugin             ## Kinematics solver for both arms
  invite_motoman_support
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
)

catkin_package(
  CATKIN_DEPENDS 
    trac_ik_kinematics_plugin 
    invite_motoman_support
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
