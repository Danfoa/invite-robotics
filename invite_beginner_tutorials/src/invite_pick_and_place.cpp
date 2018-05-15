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
  bool SIMULATION;
  ros::param::param<bool>("/invite_pick_and_place/sim", SIMULATION, true);
  ROS_INFO_COND( SIMULATION, "Tutorial Pick and Place: Operating %s robot", SIMULATION ? "Simulated":"Real");

  moveit::planning_interface::MoveGroupInterface arm_right_move_group("arm_right");
  moveit::planning_interface::MoveGroupInterface csda10f_move_group("csda10f");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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
// STEP 1:ADD WORKING OBJECT (CUP)
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
  mesh_pose.position.x = -0.10;
  mesh_pose.position.y = -0.80;
  mesh_pose.position.z = 0.77;
  mesh_pose.orientation.w= 1.0; 
  mesh_pose.orientation.x= 0.0; 
  mesh_pose.orientation.y= 0.0;
    
  //the object is added like un colission object
  glass_cup.meshes.push_back(mesh);
  glass_cup.mesh_poses.push_back(mesh_pose);
  glass_cup.operation = glass_cup.ADD;      // operations are be glass_cup.REMOVE, glass_cup.APPEND, glass_cup.MOVE 

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects;
  objects.push_back(glass_cup);
    
  // Add the collision objects into the world
  planning_scene_interface.addCollisionObjects(objects);
  
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Glass cup added into the world\nThis will be the object to manipulate", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();    
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
 

// ***********************************************************************************************
// STEP 2: PLAN TO PRE TEACH HOME POSITION

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *csda10f_joint_model_group = csda10f_move_group.getCurrentState()->getJointModelGroup("csda10f");

  // To start, we'll create a pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = csda10f_move_group.getCurrentState();

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking arm_right_move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  csda10f_move_group.setNamedTarget("home_right_arms_folded");

  bool success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
  
  visual_tools.publishAxisLabeled(mesh_pose, "mesh_pose");
  visual_tools.publishText(text_pose, "Planning to custom pre-teach home position \n Press next to perform the planned motion", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // ***********************************************************************************************
  // STEP 2: APPROACH CUP

  // If not simulating, move the robot after user has a visualized the robot motion.
  if( !SIMULATION )
    csda10f_move_group.move();

  // Get joint group state
  const robot_state::JointModelGroup *arm_right_joint_model_group = arm_right_move_group.getCurrentState()->getJointModelGroup("arm_right");

  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose approach_pose = mesh_pose;    // Use target object pose coordinates as reference
  tf::Quaternion orientation;
  // Set targer orientation as euler angles for ease of use
  //                ROLL-PITCH-YAW    [Radians]
  orientation.setRPY(M_PI , 0.0 , 0.0);
  
  approach_pose.orientation.x = orientation.x();
  approach_pose.orientation.y = orientation.y();
  approach_pose.orientation.z = orientation.z();
  approach_pose.orientation.w = orientation.w();
  approach_pose.position.z += 0.15;    // Move 15 cm above the object.
  
  arm_right_move_group.setPoseTarget(approach_pose);

  success = (arm_right_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pick Tutorial: Visualizing plan 2 (approach position) %s", success ? "SUCCESS" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO_NAMED("Pick Tutorial", "Visualizing plan 2 as trajectory line");
  visual_tools.publishAxisLabeled(approach_pose, "pose1");
  visual_tools.publishText(text_pose, "Base (Cartesian) pose coordinate goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, arm_right_joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

/*
//************************************************************************************************
// POSE TARGET


 // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = orientation.x();
  target_pose1.orientation.y = orientation.y();
  target_pose1.orientation.z = orientation.z();
  target_pose1.orientation.w = orientation.w();
  target_pose1.position.x = 0.41;  //[meters]
  target_pose1.position.y = -0.91; //[meters]
  target_pose1.position.z = 0.78;  //[meters]
  arm_right_move_group.setPlanningTime(10.0);


  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group = arm_right_move_group.getCurrentState()->getJointModelGroup("arm_right");

  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  start_state.setFromIK(joint_model_group, target_pose1);
  arm_right_move_group.setStartState(start_state);
  

  arm_right_move_group.setPoseTarget(target_pose1);
  arm_right_move_group.setPlanningTime(10.0); 
  ros::Duration(5.0).sleep();
  arm_right_move_group.move();


  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools.publishText(text_pose, "Go for the glass", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

//************************************************************************************************
// ATTACH THE OBJECT TO THE GRIPPER

   static const std::string PLANNING_GROUP2 = "right_gripper";
   moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP2);
   move_group2.attachObject(glass_cup.id);

  // Show text in Rviz of status

  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Sleep to allow MoveGroup to recieve and process the attached collision object message 
  ros::Duration(2.0).sleep();

//************************************************************************************************
// MOVE THE OBJECT


 geometry_msgs::Pose target_pose2;
 target_pose2.orientation.x = orientation.x();
 target_pose2.orientation.y = orientation.y();
 target_pose2.orientation.z = orientation.z();
 target_pose2.orientation.w = orientation.w();
 target_pose2.position.x = -0.18;      //[meters]
 target_pose2.position.y = -0.84;     //[meters]
 target_pose2.position.z = 0.8;      //[meters]
 start_state.setFromIK(joint_model_group, target_pose2);
 arm_right_move_group.setStartState(start_state);
 arm_right_move_group.setPoseTarget(target_pose2);
  
  arm_right_move_group.setNumPlanningAttempts(3);
  arm_right_move_group.setPlanningTime(10.0);  
  arm_right_move_group.move();


  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose2, "goal");
  visual_tools.publishAxisLabeled(target_pose1, "start");
  visual_tools.publishText(text_pose, "Moving..", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");



  // When done with the path constraint be sure to clear it.
  arm_right_move_group.clearPathConstraints();

//************************************************************************************************
// PLACE THE OBJECT


  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");
  move_group2.detachObject(glass_cup.id);

  // Show text in Rviz of status

  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Sleep to allow MoveGroup to recieve and process the detach collision object message //
  ros::Duration(2.0).sleep();
//************************************************************************************************
// REMOVE THE OBJECT


  // Now, let's remove the collision object from the world.
  ROS_INFO("Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(glass_cup.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  //Sleep to give Rviz time to show the object is no longer there.
  ros::Duration(1.0).sleep();
  visual_tools.deleteAllMarkers();



//***************************************************

*/

  ros::shutdown();
  return 0;
}
