/*
 * This file is part of invite_utils package.
 *
 * To-Do: Add licence description
 *
 * File name: csda10f_interface.h
 *
 * Description:
 *        This header defines the necessary classes, constants and functions to
 * easily operate the CSDA10F robot.
 *
 *        The main class is the `CSDA10F`, which on instantiation creates the
 * relevant move groups and robot variables/clients to properly control the
 * robot and its grippers.
 *
 *        This is a robot specific file, but with the appropiate changes can be
 * used with other robots.
 *
 * Author:
 *        Daniel Felipe Ordonez Apraez - daniels.ordonez@gmail.com -
 * OrdonezApraez@invite-research.com
 */
#ifndef CSDA10F_H
#define CSDA10F_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_control/robotiq_gripper_client.h>

#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/StopMotion.h>
#include <std_srvs/Trigger.h>
#include <sound_play/SoundRequest.h>
#include <robotiq_ft_sensor/sensor_accessor.h>

typedef moveit::planning_interface::MoveItErrorCode MoveItErrorCode;
typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;

typedef robotiq_2f_gripper_control::RobotiqActionClient RobotiqActionClient;

namespace invite_utils {

  namespace rvt = rviz_visual_tools;
  typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperAction
      CommandRobotiqGripperAction;

  /** Auxiliary class that handles several robot specific operations and holds
   * helpfull member variables to control and monitor the state of the robot and
   * its grippers.
   */
  class CSDA10F {
  private:
    // Max velocity in percentage from 0 to 1 (100%) of robot motions
    float max_velocity_scaling_factor;
    // Default joint orientation tolerance
    float joint_goal_orientation_tolerance = 1e-4;
    // Publisher to Robot sound generator topic
    ros::Publisher sound_pub;
    // Boolean indicating whether robot should speak or not.
    bool talkative = false;

    // Planning scene monitor and client
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    ros::ServiceClient planning_scene_diff_client_;

    // moveit::planning_interface::PlanningSceneInterfacePtr psi_;
    // Robot model instance
    robot_model::RobotModelConstPtr robot_model_;

    MoveItErrorCode error_code;

    // Service clients to enable or disable robot servos
    ros::ServiceClient robot_enabler_srv_;
    ros::ServiceClient robot_disabler_srv_;
    // Serice client to stop robot motion
    ros::ServiceClient stop_motion_srv_;
    // Service Client to command Robotiq FT300 force/torque sensors
    ros::ServiceClient left_ft300_srv_;

    ros::NodeHandlePtr nh_;

  public:
    // Move groups
    MoveGroupInterface csda10f_mg;
    MoveGroupInterface arms_mg;
    MoveGroupInterface left_arm_with_torso_mg;
    MoveGroupInterface right_arm_with_torso_mg;
    MoveGroupInterface left_arm_mg;
    MoveGroupInterface right_arm_mg;
    MoveGroupInterface torso_mg;
    // Robot status
    industrial_msgs::RobotStatus status;
    // Robot state pointer
    robot_state::RobotStatePtr current_robot_state_ptr;
    // Grippers action clients
    RobotiqActionClient* left_gripper;
    RobotiqActionClient* right_gripper;
    // Member variable to provide visual debugging
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;

    // ---------
    // TO-DO: Initialize move-groups on demand, or in a secondary thread.

    CSDA10F(float max_speed);

    // Enable robot servomotor motions
    bool enable_robot();

    // Disable robot servomotor motions
    bool disable_robot();

    // Stop robot motion
    bool stopMotion();

    // Configure the robot move group parameters before enabling motion
    void configureRobotInterface();

    bool goHome(std::string home_pose_name);

    // Set force sensor values to cero
    bool tareLeftForceSensor();

    // Enable speech synthesize
    void speak(std::string msg);

    // Update robot status message from FS100 controller
    void updateRobotStatus(
        const industrial_msgs::RobotStatusConstPtr& new_status);

    // Return the trajectory message to take the right gripper to a position where
    // the fingers have a determined distance between each other
    static trajectory_msgs::JointTrajectory getRightGripperPosture(
        float distance_between_fingers);

    // Return the trajectory message to take the left gripper to a position where
    // the fingers have a determined distance between each other
    static trajectory_msgs::JointTrajectory getLeftGripperPosture(
        float distance_between_fingers);

    // Get the Moveit Error code message for console output
    static std::string getErrorMsg(MoveItErrorCode error_code);
  };

}  // namespace invite_utils

#endif