cmake_minimum_required(VERSION 2.8.3)
project(invite_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin 
  REQUIRED
  dynamic_reconfigure
  genmsg 
  actionlib_msgs 
  actionlib
  COMPONENTS 
  roscpp 
  rospy 
  ensenso_camera 
  pcl_conversions 
  pcl_ros 
  sensor_msgs
  moveit_ros_planning_interface
  moveit_visual_tools)

find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )

generate_messages(DEPENDENCIES actionlib_msgs)

generate_dynamic_reconfigure_options(
  cfg/drum_params.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES invite_vision
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS
    EIGEN3
    moveit_ros_planning_interface
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

roslint_python(
  scripts/request_cameras_data
  scripts/request_upper_camera_data
  scripts/request_lower_camera_data
)

add_executable(concatenate_cameras_pointcloud
  src/concatenate_cameras_pointcloud.cpp
)

add_dependencies(concatenate_cameras_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(concatenate_cameras_pointcloud ${catkin_LIBRARIES})

add_executable(drum_finder
  src/drum_finder.cpp
)

add_dependencies(drum_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drum_finder ${PROJECT_NAME}_gencfg)
target_link_libraries(drum_finder ${catkin_LIBRARIES} ${gsl_LIBRARIES})

