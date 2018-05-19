/*********************************************************************
 * Software License Agreement (BSD License)
 *********************************************************************
 Author:      Alejandro Acevedo
 Modified by: Daniel Ordonez - daniels.ordonez@gmail.com    10/May/2018
*/

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
//For add object
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
// Logging capabilities
#include <ros/console.h>

using namespace Eigen;

int main(int argc, char **argv){

// Initial CONFIGURATION
// ************************************************************************************************
  ros::init(argc, argv, "invite_pick_and_place");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Read Ros parameter indicating whther we are operating a simulation or the real robot
  // Variable used for detecting whether a motion plan was succesful.
  bool success;

  // Set up move group objects 
  moveit::planning_interface::MoveGroupInterface arm_right_move_group("arm_right");
  arm_right_move_group.allowReplanning(true);     // Allow move group to re plan motions when scene changes are detected
  moveit::planning_interface::MoveGroupInterface csda10f_move_group("csda10f");
  csda10f_move_group.allowReplanning(true);       // Allow move group to re plan motions when scene changes are detected
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *csda10f_joint_model_group = csda10f_move_group.getCurrentState()->getJointModelGroup("csda10f");
  const robot_state::JointModelGroup *arm_right_joint_model_group = arm_right_move_group.getCurrentState()->getJointModelGroup("arm_right");

  // Holder for motion plans.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // RobotState instance that will hold movoe group robot states instances.
  robot_state::RobotState start_state(*arm_right_move_group.getCurrentState());

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Define visualization parameters
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // Set up tutorial text position
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 2.00; // above head of CSDA10F

  visual_tools.publishText(text_pose, "Pick and place Tutorial \n Press next to start", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

// *****************************************************************************************
// STEP 1: Go to home position.

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  csda10f_move_group.setNamedTarget("home_arms_folded");
  success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Go back to home position/n Press next to perform motion", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  csda10f_move_group.execute(my_plan);

// *****************************************************************************************
// STEP 2: Add manipulation object.
  visual_tools.publishText(text_pose, "Loading mesh model...", rvt::WHITE, rvt::XXLARGE);

  //Vector to scale
  Vector3d vectorScale(0.001, 0.001, 0.001);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject glass_cup;
  // The id of the object is used to identify it.
  glass_cup.id = "glass";

  //Path where is located le model 
  shapes::Mesh* m = shapes::createMeshFromResource("package://invite_beginner_tutorials/meshes/glass.stl", vectorScale); 
  // Define and load the mesh
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;  
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  ROS_INFO("Glass mesh loaded");

  glass_cup.meshes.resize(1);
  glass_cup.mesh_poses.resize(1);

  //Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh_pose;
  mesh_pose.position.x = -0.20;
  mesh_pose.position.y = -0.80;
  mesh_pose.position.z = 0.77;
  mesh_pose.orientation.w= 1.0; 
  mesh_pose.orientation.x= 0.0; 
  mesh_pose.orientation.y= 0.0;
    
  //The object is added like un colission object
  glass_cup.meshes.push_back(mesh);
  glass_cup.mesh_poses.push_back(mesh_pose);
  glass_cup.operation = glass_cup.ADD;      // operations are be glass_cup.REMOVE, glass_cup.APPEND, glass_cup.MOVE 

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects;
  objects.push_back(glass_cup);
    
  // Add the collision objects into the world
  planning_scene_interface.addCollisionObjects(objects);
  ros::Duration(1.5).sleep();
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Glass cup added into the world\nThis will be the object to manipulate", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();    
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
 

  // ***********************************************************************************************
  // STEP 3: APPROACH CUP
  
  // Set up target pose for right arm.
  geometry_msgs::Pose approach_pose = mesh_pose;    // Use target object pose (x,y,z) coordinates as reference.
  tf::Quaternion orientation;
  orientation.setRPY(M_PI , 0.0 , 0.0);
  approach_pose.orientation.x = orientation.x();
  approach_pose.orientation.y = orientation.y();
  approach_pose.orientation.z = orientation.z();
  approach_pose.orientation.w = orientation.w();
  approach_pose.position.z += 0.13;                 // Move 15 cm above the target object object.
  
  arm_right_move_group.setPoseTarget(approach_pose);
  // Compute motion plan 
  success = (arm_right_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pick Tutorial: picking movements %s", success ? "SUCCESS" : "FAILED");
  
  // Visualizing plan
  // ^^^^^^^^^^^^^^^^^
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(approach_pose, "mesh origin");
  visual_tools.publishText(text_pose, "Planning for object grasping/nPress next to perform motion on the real robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, arm_right_joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
  
  // Once user allow it, exceute the planned trajectory
  ROS_INFO("Performing motion on real robot...");
  arm_right_move_group.execute(my_plan);

//************************************************************************************************
// STEP 4: ATTACH THE OBJECT TO THE GRIPPER

  moveit::planning_interface::MoveGroupInterface right_gripper_mg("right_gripper");
  right_gripper_mg.attachObject(glass_cup.id);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Sleep to allow MoveGroup to recieve and process the attached collision object message 
  // ros::Duration(1.0).sleep();

//************************************************************************************************
// STEP 5: PLACE THE OBJECT

  visual_tools.publishText(text_pose, "Planning with kinematic constraints...please wait", rvt::WHITE, rvt::XLARGE);
  ROS_DEBUG("Planning with kinematic constraints...please wait");

  // Create a path constraint to keep the cup facing up, to avoid spilling liquids.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "arm_right_link_tcp";               // Gripper base rigidly align to the TCP 
  ocm.header.frame_id = "base_link";
  ocm.orientation = approach_pose.orientation;            
  ocm.absolute_x_axis_tolerance = 25*(M_PI/180);
  ocm.absolute_y_axis_tolerance = 25*(M_PI/180);
  ocm.absolute_z_axis_tolerance = 360*(M_PI/180);
  // A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
  ocm.weight = 0.7;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  csda10f_move_group.setPathConstraints(test_constraints);

  // Set drop position *******************************
  geometry_msgs::Pose drop_pose = approach_pose;
  drop_pose.position.x += 0.7;              //[meters]
  drop_pose.position.y += 0;                //[meters]
  drop_pose.position.z += 0;                //[meters]
  csda10f_move_group.setPoseTarget(drop_pose, "arm_right_link_tcp");

  // Planning with constraints can be slow because every sample must call an inverse 
  // kinematics solver.
  // Lets increase the planning time from the default 10 seconds to be sure the planner has 
  // enough time to succeed.
  csda10f_move_group.setPlanningTime(20.0);
  csda10f_move_group.setMaxAccelerationScalingFactor(0.5); 
  csda10f_move_group.setMaxVelocityScalingFactor(0.2);
  csda10f_move_group.setNumPlanningAttempts(3);       //Even do it will take time replanning improves results... we are not in a hurry.

  // Plan the motion wiht constraints.
  success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("Pick&Place Tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(approach_pose, "start");
  visual_tools.publishAxisLabeled(drop_pose, "goal");
  visual_tools.publishText(text_pose, "Perform motion of the cup mantaining orientation to avoid liquid spilling/nPress next to perform motion on real robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Once user allow it, exceute the planned trajectory
  ROS_INFO("Performing motion on real robot...");
  csda10f_move_group.execute(my_plan);
 
  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");
  move_group2.detachObject(glass_cup.id);

  // When done with the path constraint be sure to clear it.
  csda10f_move_group.clearPathConstraints();

  //************************************************************************************************
  // STEP 6: PLAN TO PRE TEACH HOME POSITION

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  csda10f_move_group.setNamedTarget("home_arms_folded");
  success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Go back to home position/n Press next to perform motion", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  csda10f_move_group.execute(my_plan);
  ros::shutdown();
  return 0;
}
