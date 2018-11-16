cmake_minimum_required(VERSION 2.8.3)
project(invite_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin 
  REQUIRED
  genmsg 
  actionlib_msgs 
  actionlib
  COMPONENTS 
  roscpp 
  rospy 
  ensenso_camera 
  pcl_conversions 
  pcl_ros 
  sensor_msgs)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES invite_vision
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

roslint_python(
  scripts/request_cameras_data
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
target_link_libraries(drum_finder ${catkin_LIBRARIES} )

# # Tutorial executables.
# add_executable(camera_calibration_pose_generator 
#   src/camera_calibration_pose_generator.cpp
# )
# target_link_libraries(camera_calibration_pose_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# install(TARGETS camera_calibration_pose_generator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
