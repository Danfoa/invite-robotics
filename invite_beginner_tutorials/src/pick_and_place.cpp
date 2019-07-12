#include <ros/ros.h>
#include <invite_utils/csda10f_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <tf2_eigen/tf2_eigen.h>


namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualToolsPtr visual_tools;

int main(int argc, char** argv) {
  ros::init(argc, argv, "cartesian_task_planning");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Start CSDA10F robot interface
  // ----------------------------------------------------------------
  invite_utils::CSDA10F csda10f(1.0);  // Start CSDA10F robot interface

  csda10f.right_gripper->open();
  csda10f.left_gripper->open();

  csda10f.visual_tools->removeAllCollisionObjects();
  csda10f.visual_tools->loadRemoteControl();  // Optional UI control
  // ----------------------------------------------------------------------------------------------

  // Add Collision object ----------------------------------------
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  moveit_msgs::CollisionObject rod_obj;

  rod_obj.header.frame_id = "base_link";
  rod_obj.id = "rod";

  /* Define the primitive and its dimensions. */
  rod_obj.primitives.resize(1);
  rod_obj.primitives[0].type = rod_obj.primitives[0].BOX;
  rod_obj.primitives[0].dimensions.resize(3);
  rod_obj.primitives[0].dimensions[0] = 0.03;
  rod_obj.primitives[0].dimensions[1] = 0.03;
  rod_obj.primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  geometry_msgs::Pose rod_obj_pose;
  rod_obj_pose.position.x = 0.8;
  rod_obj_pose.position.y = -0.6;
  rod_obj_pose.position.z = 1.13;
  rod_obj.primitive_poses.push_back(rod_obj_pose);

  collision_objects.push_back(rod_obj);
  // Add Table Collision object ----------------------------------------
  moveit_msgs::CollisionObject tables_obj;

  tables_obj.header.frame_id = "base_link";
  tables_obj.id = "table";

  /* Define the primitive and its dimensions. */
  tables_obj.primitives.resize(1);
  tables_obj.primitives[0].type = tables_obj.primitives[0].BOX;
  tables_obj.primitives[0].dimensions.resize(3);
  tables_obj.primitives[0].dimensions[0] = 1.0;
  tables_obj.primitives[0].dimensions[1] = 2.0;
  tables_obj.primitives[0].dimensions[2] = 0.02;
  /* Define the pose of the object. */
  geometry_msgs::Pose tables_obj_pose;
  tables_obj_pose.position.x = 1.1;
  tables_obj_pose.position.y = 0.0;
  tables_obj_pose.position.z = 1.0;
  tables_obj.primitive_poses.push_back(tables_obj_pose);

  collision_objects.push_back(tables_obj);
  planning_scene_interface.applyCollisionObjects(collision_objects);
  // ----------------------------------------------------------------------------------------------

  // Create Grap Message
  // --------------------------------------------------------------------------
  moveit_msgs::Grasp grasp;
  // Get the gripper grasping posture by providing the desired distance between
  // fingers in [m]
  grasp.grasp_posture = csda10f.getRightGripperPosture(0.04);
  grasp.pre_grasp_posture = csda10f.getRightGripperPosture(0.085);              // Open
  // Configure pre grasp approach motion
  grasp.pre_grasp_approach.direction.header.frame_id = "arm_right_link_tcp";    // Motion relative to tool frame
  grasp.pre_grasp_approach.direction.vector.z = 1.0;  // Approach from above
  grasp.pre_grasp_approach.min_distance = 0.05;
  grasp.pre_grasp_approach.desired_distance = 0.10;
  // Configure post grasp retreat
  grasp.post_grasp_retreat.direction.header.frame_id = "base_link";             // Motion relative to base frame 
  grasp.post_grasp_retreat.direction.vector.z = 1.0;  // Retreat from above
  grasp.post_grasp_retreat.min_distance = 0.05;
  grasp.post_grasp_retreat.desired_distance = 0.20;
  // Configure grasping pose
  grasp.grasp_pose.header.frame_id = "base_link";
  grasp.grasp_pose.pose.position = rod_obj_pose.position;
  grasp.allowed_touch_objects = std::vector<std::string>{"table","rod"};
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 110* M_PI / 180, 0.0);
  grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);

  csda10f.visual_tools->publishAxisLabeled(grasp.grasp_pose.pose, "grasp pose",
                                           rvt::scales::SMALL);
  csda10f.visual_tools->publishGrasps(std::vector<moveit_msgs::Grasp>{grasp},
      csda10f.current_robot_state_ptr->getJointModelGroup("right_gripper"));

  Eigen::Affine3d current_pose, target_pose;
  tf2::fromMsg(grasp.grasp_pose.pose, current_pose);

  target_pose.translation()[2] = -0.1;
  target_pose.linear().setIdentity();
  target_pose = current_pose * target_pose;

  csda10f.visual_tools->publishAxisLabeled(tf2::toMsg(target_pose), "pre_approach pose",
                                           rvt::scales::SMALL);
  csda10f.visual_tools->trigger();
  // ----------------------------------------------------------------------------------------------

  // Create Place Message
  // --------------------------------------------------------------------------
  moveit_msgs::PlaceLocation place_msg;
  place_msg.post_place_posture = csda10f.getRightGripperPosture(0.085);
  place_msg.allowed_touch_objects = std::vector<std::string>{};
  place_msg.pre_place_approach.direction.header.frame_id = "base_link";
  place_msg.pre_place_approach.direction.vector.z = 1.0;
  place_msg.pre_place_approach.desired_distance = 0.1;
  place_msg.pre_place_approach.min_distance = 0.01;

  place_msg.post_place_retreat.direction.header.frame_id = "arm_right_link_tcp";
  place_msg.post_place_retreat.direction.vector.z = -1.0;
  place_msg.post_place_retreat.desired_distance = 0.05;
  place_msg.post_place_retreat.min_distance = 0.005;

  place_msg.place_pose.pose = grasp.grasp_pose.pose;
  place_msg.place_pose.header.frame_id = "base_link";
  place_msg.id = "0";
  // place_msg.place_pose.pose.position.x = 0.8;
  // place_msg.place_pose.pose.position.y = -0.6;
  // place_msg.place_pose.pose.position.z += 0.2;
  // orientation.setRPY(0.0, 110* M_PI / 180, 0);
  // place_msg.place_pose.pose.orientation = tf2::toMsg(orientation);

  csda10f.visual_tools->publishAxisLabeled(place_msg.place_pose.pose, "place pose",
                                           rvt::scales::SMALL);
  csda10f.visual_tools->trigger();

  // Plan and perform grasp motion
  // ----------------------------------------------------------------
  csda10f.right_arm_with_torso_mg.setSupportSurfaceName("table");
  while (!ros::isShuttingDown()) {

    csda10f.right_gripper->open(true);

    MoveItErrorCode error_code = csda10f.right_arm_with_torso_mg.pick("rod", grasp);
    ROS_WARN("Pick executon returned: %s",
            invite_utils::CSDA10F::getErrorMsg(error_code).c_str());

    error_code = csda10f.right_arm_with_torso_mg.place("rod", place_msg.place_pose);
    ROS_WARN("Place executon returned: %s",
            invite_utils::CSDA10F::getErrorMsg(error_code).c_str());
    
    csda10f.right_arm_with_torso_mg.detachObject("rod");
    // planning_scene_interface.applyCollisionObjects(collision_objects);
  }
  ros::shutdown();
  return 0;
}