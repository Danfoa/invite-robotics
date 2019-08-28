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

  COMPONENTS 
    roscpp 
    rospy 
    ensenso_camera 
    ensenso_camera_msgs
    pcl_conversions 
    pcl_ros 
    sensor_msgs
    std_msgs
)

#add_message_files(
#  DIRECTORY 
#    msg
#  FILES
#)

# generate_messages(DEPENDENCIES std_msgs)
  
# generate_dynamic_reconfigure_options()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES invite_vision
  CATKIN_DEPENDS ensenso_camera_msgs
  DEPENDS
    EIGEN3
)
include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#roslint_python(
#  scripts/request_cameras_data
#  scripts/request_upper_camera_data
#  scripts/request_lower_camera_data
#)

add_executable(concatenate_cameras_pointcloud  src/concatenate_cameras_pointcloud.cpp)
add_dependencies(concatenate_cameras_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(concatenate_cameras_pointcloud ${catkin_LIBRARIES})
