cmake_minimum_required(VERSION 2.8.3)
project(invite_beginner_tutorials)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED 
 COMPONENTS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib 
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Tutorial executables.
add_executable(invite_move_group_interface src/invite_move_group_interface.cpp)
target_link_libraries(invite_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS invite_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(invite_pick_and_place src/invite_pick_and_place.cpp)
target_link_libraries(invite_pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS invite_pick_and_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/invite_move_group_python_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
