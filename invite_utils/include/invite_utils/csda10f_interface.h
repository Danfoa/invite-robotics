/*
 * This file is part of invite_utils package.
 *
 * To-Do: Add licence description
 * 
 * File name: csda10f_interface.h
 * 
 * Description: 
 *        This header defines the necessary classes, constants and functions to easily operate the CSDA10F robot.
 *        
 *        The main class is the `CSDA10F`, which on instantiation creates the relevant move groups and robot 
 *        variables/clients to properly control the robot and its grippers. 
 * 
 *        This is a robot specific file, but with the appropiate changes can be used with other robots.
 * 
 * Author: 
 *        Daniel Felipe Ordonez Apraez - daniels.ordonez@gmail.com - OrdonezApraez@invite-research.com  
 */
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
#include <std_srvs/Trigger.h>
#include <sound_play/SoundRequest.h>


typedef moveit::planning_interface::MoveItErrorCode MoveItErrorCode;
typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;

typedef robotiq_2f_gripper_control::RobotiqActionClient RobotiqActionClient;

namespace invite_utils{

    namespace rvt = rviz_visual_tools;
    typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperAction CommandRobotiqGripperAction;

    /** Auxiliary class that handles several robot specific operations and holds helpfull member
     * variables to control and monitor the state of the robot and its grippers.
     */
    class CSDA10F{
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
          
          // Action clients to enable or disable robot operation 
          ros::ServiceClient enabler_;
          ros::ServiceClient disabler_;

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

          CSDA10F(float max_speed = 0.1) : csda10f_mg("csda10f"), 
                                            arms_mg("arms"),
                                            torso_mg("torso"),
                                            left_arm_mg("arm_left"),
                                            left_arm_with_torso_mg("arm_left_with_torso"),
                                            right_arm_mg("arm_right"),
                                            right_arm_with_torso_mg("arm_right_with_torso"){
            nh_.reset(new ros::NodeHandle());

            // Robot configuration
            max_velocity_scaling_factor = max_speed;
            // Start gripper action servers.
            left_gripper = new RobotiqActionClient("/left_gripper/command_robotiq_action", true);
            right_gripper = new RobotiqActionClient("/right_gripper/command_robotiq_action", true);
            // Check current robot state is obtainable.
            if ( !csda10f_mg.startStateMonitor() ) {
              ROS_FATAL("Robot state monitor was not created, restart robot firmware");
              throw("Robot state monitor was not created, restart robot firmware");
            }
            current_robot_state_ptr = csda10f_mg.getCurrentState(3.0);
            configureRobotInterface();
            // Connect to YASKAWAS robot state messages  
            ros::Subscriber sub = nh_->subscribe<industrial_msgs::RobotStatus>("/robot_status", 
                                                                              1,
                                                                              &CSDA10F::updateRobotStatus,
                                                                              this);
            //  Generate client to enable/dissable robot servo-power / motions
            enabler_ = nh_->serviceClient<std_srvs::Trigger>("/robot_enable", true);
            disabler_ = nh_->serviceClient<std_srvs::Trigger>("/robot_enable", true);
            enable_robot();

            ROS_WARN("Robot ready for motion at %.1f%% of its max speed", max_velocity_scaling_factor * 100);

            // Publishers, subscribers, visual tools and services config
            sound_pub = nh_->advertise<sound_play::SoundRequest>("/robotsound", 3);
            talkative = true;
            psm_.reset( new planning_scene_monitor::PlanningSceneMonitor("robot_description") );
            // psm_->requestPlanningSceneState();
            psm_->publishDebugInformation(true);
            psm_->startSceneMonitor("/move_group/monitored_planning_scene");
            
            planning_scene_diff_client_ = nh_->serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
            planning_scene_diff_client_.waitForExistence();
            // Visualization functions 
            visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/rviz_visual_tools", psm_));
            // visual_tools->loadPlanningSceneMonitor();
            visual_tools->loadMarkerPub(true);
            visual_tools->loadRobotStatePub("/display_robot_state");
            visual_tools->loadTrajectoryPub("/move_group/display_planned_path");
            visual_tools->loadSharedRobotState();
            visual_tools->enableBatchPublishing();
            visual_tools->deleteAllMarkers();
            // visual_tools->hideRobot();
            visual_tools->trigger();
            // visual_tools->setManualSceneUpdating();
            
            // Load default collision scene if path is given 
            std::string collision_scene_path, param_key;
            if (nh_->searchParam("bag_handling/collision_scene", param_key)) {
              nh_->getParam(param_key, collision_scene_path);
              ROS_WARN("Loading collision scene (%s)", collision_scene_path.c_str());
              visual_tools->loadCollisionSceneFromFile(collision_scene_path);
            }else
              ROS_WARN("No collision scene loaded");

            visual_tools->trigger();
            // Planning Scene configuration
            robot_model_ = psm_->getRobotModel();
          }

          // Enable robot servomotor motions
          bool enable_robot() {
            if (enabler_) {
              ROS_WARN("Enabling Robot");
              std_srvs::Trigger cmd;
              enabler_.call(cmd);
              if(cmd.response.success)
                ROS_WARN("Robot is now enabled for motion");
              else
                ROS_ERROR("Robot was not enabled: %s", cmd.response.message.c_str());
            }
          }

          // Disable robot servomotor motions
          bool disable_robot() {
            if (disabler_) {
              ROS_DEBUG("Disabling Robot");
              std_srvs::Trigger cmd;
              enabler_.call(cmd);
              if(!cmd.response.success)
                ROS_ERROR("Robot was not disabled: %s", cmd.response.message.c_str());
            }
          }

          bool goHome(std::string home_pose_name = "home") {
            if (!status.motion_possible.val == true)
              enable_robot();
            if (talkative)
              speak("Going to " + home_pose_name + " position");
            // Take left arm home'
            csda10f_mg.setNamedTarget(home_pose_name);
            error_code = csda10f_mg.move();
            if (error_code != MoveItErrorCode::SUCCESS){
              ROS_ERROR("Motion to position (%s) impossible, error code: (%d) %s", 
                          home_pose_name.c_str(), 
                          (int)error_code.val,
                          getErrorMsg(error_code).c_str()
                          );
              return false;
            }
            ros::Duration(0.4).sleep();
            return true;
          }

          void configureRobotInterface() {
            // Allow replanning
            // csda10f_mg.allowReplanning(true);
            // arms_mg.allowReplanning(true);
            // left_arm_with_torso_mg.allowReplanning(true);
            // right_arm_with_torso_mg.allowReplanning(true);
            // left_arm_mg.allowReplanning(true);
            // right_arm_mg.allowReplanning(true);
            // torso_mg.allowReplanning(true);

            // Set Velocity Scaling Factor
            csda10f_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
            arms_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
            left_arm_with_torso_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
            right_arm_with_torso_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
            left_arm_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
            right_arm_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
            torso_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor * 2);
            
            // Set Joint orientation tolerance (This is needed to cope with the `joint_trajectory_streamer` start
            // position tolerance limit `start_pos_tol_`
            // csda10f_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            // arms_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            // left_arm_with_torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            // right_arm_with_torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            // left_arm_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            // right_arm_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            // torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            speak("I am ready");
          }
          
          void speak(std::string msg) {
            if (!talkative) {
              // ROS_INFO(msg);
              return;
            }
            sound_play::SoundRequest req;
            req.sound = req.SAY;
            req.command = req.PLAY_ONCE;
            req.volume = 1.0;
            req.arg = msg;
            ROS_INFO("Speaking : %s" , msg.c_str());
            sound_pub.publish(req);
          }

          void updateRobotStatus( const industrial_msgs::RobotStatusConstPtr& new_status) {
              if (new_status->in_error.val == industrial_msgs::TriState::ON )
                ROS_ERROR("Robot error: alarm % is active", new_status->error_code);
              if (new_status->in_motion.val == false && status.in_motion.val == true){
                current_robot_state_ptr = csda10f_mg.getCurrentState();
                ROS_INFO_STREAM("-- Robot State: Robot has just stopped its motion " << current_robot_state_ptr );
              }
              if (new_status->motion_possible.val == false && status.motion_possible.val == true)
                ROS_ERROR("Robot motion was disabled");
              this->status = *new_status;
          }

        // Return the trajectory message to take the right gripper to a position where the fingers
        // have a determined distance between each other
        static trajectory_msgs::JointTrajectory getRightGripperPosture(float distance_between_fingers) {
          trajectory_msgs::JointTrajectory posture;
          posture.header.stamp = ros::Time::now();
          posture.joint_names = std::vector<std::string>{"right_gripper_finger_joint"};
          trajectory_msgs::JointTrajectoryPoint close_state;

          float joint_position = 0.8 - (0.8/0.085 * distance_between_fingers);
          joint_position = joint_position > 0.8 ? 0.8 : joint_position;
          joint_position = joint_position < 0.0 ? 0.0 : joint_position;

          close_state.positions.push_back(joint_position);
          close_state.velocities.push_back(0.05f);
          close_state.effort.push_back(10.0);
          close_state.time_from_start = ros::Duration(4.5);
          posture.points.push_back(close_state);
          return posture;
        }

        // Return the trajectory message to take the left gripper to a position where the fingers
        // have a determined distance between each other
        static trajectory_msgs::JointTrajectory getLeftGripperPosture(float distance_between_fingers) {
          trajectory_msgs::JointTrajectory posture;
          posture.header.stamp = ros::Time::now();
          posture.joint_names = std::vector<std::string>{"left_gripper_finger_joint"};
          trajectory_msgs::JointTrajectoryPoint close_state;
          
          float joint_position = 0.8 - (0.8/0.085 * distance_between_fingers);
          joint_position = joint_position > 0.8 ? 0.8 : joint_position;
          joint_position = joint_position < 0.0 ? 0.0 : joint_position;

          close_state.positions.push_back(joint_position);
          close_state.velocities.push_back(0.05f);
          close_state.effort.push_back(10.0);
          close_state.time_from_start = ros::Duration(4.5);
          posture.points.push_back(close_state);
          return posture;
        }


        static std::string getErrorMsg(MoveItErrorCode error_code){
          std::string msg;
          switch(error_code.val){
            case 1: msg = "Success"; break;
            case 99999: msg = "Failure"; break;              
            case -1: msg = "Plannning failed"; break;              
            case -2: msg = "Invalid motion plan"; break;              
            case -3: msg = "Motion plan invalidated by environment change"; break;              
            case -4: msg = "Control failed"; break;              
            case -5: msg = "Unable to aquire sensor data"; break;              
            case -6: msg = "Time out reach"; break;              
            case -7: msg = "Preemted"; break;              
            case -10: msg = "Start state is in collision"; break;              
            case -11: msg = "Start state violates path constraints"; break;              
            case -12: msg = "Goal state is in collision"; break;              
            case -13: msg = "Goal state violates path constraints"; break;              
            case -14: msg = "Goal constraints violated"; break;              
            case -15: msg = "Invalid group name"; break;              
            case -16: msg = "Invalid goal constraints"; break;              
            case -17: msg = "Invalid robot state"; break;              
            case -18: msg = "Invalid link name"; break;              
            case -19: msg = "Invalid object name"; break;              
            case -21: msg = "Frame transform failure"; break;              
            case -22: msg = "Collision checking unavailable"; break;              
            case -23: msg = "Robot state stale"; break;              
            case -24: msg = "Sensor info stale"; break;              
            case -31: msg = "No Inverse Kinematics solution found"; break;
            default: msg = "Unknown error code";              
          };
          return msg;
        }

    };

}