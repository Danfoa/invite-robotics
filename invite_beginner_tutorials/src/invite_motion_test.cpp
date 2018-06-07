#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>
// Logging capabilities
#include <ros/console.h>

typedef  moveit::planning_interface::MoveItErrorCode MoveItErrorCode;

int main(int argc, char **argv){
  ros::init(argc, argv, "invite_motion_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

    // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }


  static const std::string PLANNING_GROUP = "arm_right";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  MoveItErrorCode motion_result_code;
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();    
  text_pose.translation().z() = 2.0; // above head of CSDA10F
  
  // Wait for loading 
  visual_tools.publishText(text_pose, "Wait while everything is set up....", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();

  /* Sleep to allow MoveGroup to recieve and process the attached collision object message */
  ros::Duration(8.0).sleep();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  visual_tools.publishText(text_pose, "Motion test Demo\nPress next to start", rvt::WHITE, rvt::XXLARGE);
  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());



  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create a pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  move_group.setStartStateToCurrentState();
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // Hint: Joint numbering starts from 0
  joint_group_positions[6] = M_PI/2;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Planning", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform planned motion on robot
  motion_result_code = move_group.execute(my_plan);
  success = motion_result_code == MoveItErrorCode::SUCCESS;
  ROS_WARN("Joint Space Motion: %s\n", success ? "Was Succsessfull" : "Failed");
  ROS_WARN_COND(!success, "Joint Space Motion failed with error code %d ", motion_result_code.val);

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  // Create a quaternion instance as it is required by 'geometry_msgs::Pose'
  tf::Quaternion orientation;
  // Set targer orientation as euler angles for ease of use
  //                ROLL-PITCH-YAW    [Radians]
  orientation.setRPY(0.0 , M_PI/2 , 0.0);
  target_pose1.orientation.x = orientation.x();
  target_pose1.orientation.y = orientation.y();
  target_pose1.orientation.z = orientation.z();
  target_pose1.orientation.w = orientation.w();
  target_pose1.position.x = 0.7;      //[meters]
  target_pose1.position.y = -0.5;     //[meters]
  target_pose1.position.z = 1.3;      //[meters]
  move_group.setPoseTarget(target_pose1);

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.02;
  primitive.dimensions[2] = 0.4;

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.65;
  box_pose.position.y = -0.75;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();

  // Sleep to allow MoveGroup to recieve and process the collision object message
  ros::Duration(1.0).sleep();

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setPoseTarget(target_pose1);

  success = (move_group.plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 3 (pose goal move with colision avoidance) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Motion planning with collision avoidance", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform planned motion on robot
  motion_result_code = move_group.execute(my_plan);
  success = motion_result_code == MoveItErrorCode::SUCCESS;
  ROS_WARN("Collision avoidance motion: %s \n", success ? "Was Succsessfull" : "Failed");
  ROS_WARN_COND(!success, "Joint Space Motion failed with error code %d ", motion_result_code.val);

  // Now, let's remove the collision object from the world.
  ROS_INFO("Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();

  /* Sleep to give Rviz time to show the object is no longer there.*/
  ros::Duration(1.0).sleep();


  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "arm_right_link_tcp";   // Gripper base rigidly align to the TCP 
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1;                  // Mantain orientation minuz Yaw.
  ocm.absolute_x_axis_tolerance = 15 * (M_PI/180);
  ocm.absolute_y_axis_tolerance = 15 * (M_PI/180);
  ocm.absolute_z_axis_tolerance = 2* M_PI;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  //   move_group.setPathConstraints(test_constraints);

  //Use the same orientation as the previous step
  
  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  target_pose1.position.x = 0.75;      //[meters]
  target_pose1.position.y = -0.8;      //[meters]
  target_pose1.position.z = 1.4;       //[meters]
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse 
  // kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has 
  // enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Motion planning with kinematics constraints\n(Move mantaining orientation except Yaw angle)\n(Usefull when handling containers with liquids)", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

 // Perform planned motion on robot
  motion_result_code = move_group.execute(my_plan);
  success = motion_result_code == MoveItErrorCode::SUCCESS;
  ROS_WARN("Motion planning with Kinematic constraints: %s \n", success ? "Was Succsessfull" : "Failed");
  ROS_WARN_COND(!success, "Joint Space Motion failed with error code %d ", motion_result_code.val);

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();
  
  // Cartesian Paths
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start_pose2 = move_group.getCurrentPose("arm_right_link_tcp").pose;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // up 

  target_pose3.position.y += 0.20;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.y += 0.10;
  target_pose3.position.x += 0.20;
  target_pose3.position.z -= 0.30;
  waypoints.push_back(target_pose3);  // down

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Space Motion \n(Linear motion at 10% speed)", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "point" + std::to_string(i), rvt::MEDIUM);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
 
   // Perform planned motion on robot
  motion_result_code = move_group.execute(my_plan);
  success = motion_result_code == MoveItErrorCode::SUCCESS;
  ROS_WARN("Cartesian path motion: %s \n", success ? "Was Succsessfull" : "Failed");
  ROS_WARN_COND(!success, "Joint Space Motion failed with error code %d ", motion_result_code.val);

  // Self collision avoidance pose goals
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First define a new group for addressing the two arms.
  static const std::string PLANNING_GROUP2 = "arm_left";
  moveit::planning_interface::MoveGroupInterface arm_left_move_group(PLANNING_GROUP2);

  // Define two separate pose goals, one for each end-effector. Note that
  // we are reusing the goal for the right arm above
  orientation.setRPY(0.0 , M_PI/2 , -M_PI/2);
  target_pose3.orientation.x = orientation.x();
  target_pose3.orientation.y = orientation.y();
  target_pose3.orientation.z = orientation.z();
  target_pose3.orientation.w = orientation.w();
  target_pose3.position.z -= 0.15;      //[meters]
  target_pose3.position.y += 0.00;       //[meters]
  target_pose3.position.x -= 0.30;      //[meters]

  arm_left_move_group.setPoseTarget(target_pose3, "arm_left_link_tcp");

  // Now, we can plan and visualize
  moveit::planning_interface::MoveGroupInterface::Plan right_arm_plan;

  success = (arm_left_move_group.plan(right_arm_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (dual arm plan) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose3, "Pose-Left-Arm");
  visual_tools.publishText(text_pose, "Self collision avoidance", rvt::WHITE, rvt::XXLARGE);
  joint_model_group = arm_left_move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);
  visual_tools.publishTrajectoryLine(right_arm_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
    
  // Perform planned motion on robot
  motion_result_code = arm_left_move_group.execute(right_arm_plan);
  success = motion_result_code == MoveItErrorCode::SUCCESS;
  ROS_WARN("Self collision avoidance planning: %s \n", success ? "Was Succsessfull" : "Failed");
  ROS_WARN_COND(!success, "Joint Space Motion failed with error code %d ", motion_result_code.val);

  // END_TUTORIAL
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "End of tutorial...have a happy day :)", rvt::WHITE, rvt::XXLARGE);
  visual_tools.trigger();

  ros::shutdown();
  return 0;
}