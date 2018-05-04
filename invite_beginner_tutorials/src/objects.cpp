
    co.mesh_poses[0].orientation.z= 0.0;

    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;/* Author: Alejandro Acevedo
/* 
This code show how to pick and place objects to the scenario in rviz
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
using namespace Eigen;

int main(int argc, char **argv)
{


//CONFIGURATION INITIAL
//************************************************************************************************
  ros::init(argc, argv, "object_add");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm_right";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

 // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();


  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.55; // above head of CSDA10F
  visual_tools.publishText(text_pose, "Pick and place Tutorial", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
//************************************************************************************************
//ADD OBJECT

  //Vector to scale
   Vector3d vectorScale(0.001, 0.001, 0.001);
  
  // Define a collision object ROS message.
   moveit_msgs::CollisionObject co;

  // The id of the object is used to identify it.
    co.id = "glass";

  //Path where is located le model 
    shapes::Mesh* m = shapes::createMeshFromResource("package://invite_beginner_tutorials/meshes/glass.stl", vectorScale); 
    ROS_INFO("Glass mesh loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;  
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    co.meshes.resize(1);
    co.mesh_poses.resize(1);

   //Define a pose for the object (specified relative to frame_id)
    co.mesh_poses[0].position.x = 0.41;
    co.mesh_poses[0].position.y = -0.94;
    co.mesh_poses[0].position.z = 0.77;
    co.mesh_poses[0].orientation.w= 1.0; 
    co.mesh_poses[0].orientation.x= 0.0; 
    co.mesh_poses[0].orientation.y= 0.0;
    co.mesh_poses[0].orientation.z= 0.0;
    
    //the object is added like un colission object
    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;

    std::vector<moveit_msgs::CollisionObject> object;
    object.push_back(co);
    
    // Add the collision object into the world
    planning_scene_interface.addCollisionObjects(object);
    
    visual_tools.deleteAllMarkers();
    ros::Duration(3.0).sleep();  
    visual_tools.publishText(text_pose, "Glass added into the world", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();    
    visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
//************************************************************************************************
    
//STEP 2: POSE INITIAL AND CONSTRAIN


  // To start, we'll create a pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Move to a previous pose pick to evite others paths with future colissions
  move_group.setNamedTarget("pick_right");
  ros::Duration(5.0).sleep();
  move_group.move();


  // Create a quaternion instance as it is required by 'geometry_msgs::Pose'
  tf::Quaternion orientation;
  // Set targer orientation as euler angles for ease of use
  //                ROLL-PITCH-YAW    [Radians]
  orientation.setRPY(M_PI, 0 ,0);



  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "arm_right_link_tcp";   // Gripper base rigidly align to the TCP 
  ocm.header.frame_id = "base_link";
  ocm.orientation.x = orientation.x();
  ocm.orientation.y = orientation.y();
  ocm.orientation.w = orientation.w();                // Mantain orientation minuz Yaw.
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = 0.2;
  ocm.weight = 1.0;

    // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  robot_state::RobotState start_state(*move_group.getCurrentState());
  ros::Duration(3.0).sleep();
  

 
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
  move_group.setPlanningTime(10.0);

  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  start_state.setFromIK(joint_model_group, target_pose1);
  move_group.setStartState(start_state);
  

  move_group.setPoseTarget(target_pose1);
  ros::Duration(3.0).sleep();
  move_group.move();


  // We can also visualize the plan as a line with markers in Rviz.
  visual_tools.publishText(text_pose, "Go for the glass", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

//************************************************************************************************
// ATTACH THE OBJECT TO THE GRIPPER

   static const std::string PLANNING_GROUP2 = "right_gripper";
   moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP2);
   move_group2.attachObject(co.id);

  // Show text in Rviz of status

  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
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
 move_group.setStartState(start_state);
 move_group.setPoseTarget(target_pose2);
  
  move_group.setNumPlanningAttempts(3);
  move_group.setPlanningTime(10.0);  
  move_group.move();


  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose2, "goal");
  visual_tools.publishAxisLabeled(target_pose1, "start");
  visual_tools.publishText(text_pose, "Moving..", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");



  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

//************************************************************************************************
// PLACE THE OBJECT


  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");
  move_group2.detachObject(co.id);

  // Show text in Rviz of status

  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Sleep to allow MoveGroup to recieve and process the detach collision object message //
  ros::Duration(2.0).sleep();
//************************************************************************************************
// REMOVE THE OBJECT


  // Now, let's remove the collision object from the world.
  ROS_INFO("Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(co.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  //Sleep to give Rviz time to show the object is no longer there.
  ros::Duration(1.0).sleep();
  visual_tools.deleteAllMarkers();



//***************************************************



  ros::shutdown();
  return 0;
}
    std::vector<moveit_msgs::CollisionObject> vec;
    vec.push_back(co);
    ROS_INFO("glass added into the world");
    current_scene.addCollisionObjects(vec);
//***************************************************


  ros::shutdown();
  return 0;
}
