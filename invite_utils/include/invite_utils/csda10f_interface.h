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

    class CSDA10F{
        private:        
          float max_velocity_scaling_factor;  
          float joint_goal_orientation_tolerance = 1e-4;
          // Publisher to Robot sound generator topic
          ros::Publisher sound_pub;
          // Boolean indicating whether robot should speak or not.
          bool talkative = false; 

          planning_scene_monitor::PlanningSceneMonitorPtr psm_;
          ros::ServiceClient planning_scene_diff_client_;
          // moveit::planning_interface::PlanningSceneInterfacePtr psi_;
          robot_model::RobotModelConstPtr robot_model_;

          MoveItErrorCode error_code;
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
          // Grippers
          RobotiqActionClient* left_gripper;
          RobotiqActionClient* right_gripper;

          std::vector<std::vector<double>> invalid_states = {};

          moveit_visual_tools::MoveItVisualToolsPtr visual_tools;

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
            
            std::string collision_scene_path, param_key;
            if (nh_->searchParam("bag_handling/collision_scene", param_key)) {
              nh_->getParam(param_key, collision_scene_path);
              ROS_WARN("Loading collision scene (%s)", collision_scene_path.c_str());
              // visual_tools->loadCollisionSceneFromFile(collision_scene_path);
            }else
              ROS_WARN("No collision scene loaded");

            visual_tools->trigger();
            // Planning Scene configuration
            robot_model_ = psm_->getRobotModel();
          }

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
            left_arm_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor / 2);
            right_arm_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor / 2);
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
          
          bool validateIKSolution( robot_state::RobotState* robot_state, 
                                        const robot_state::JointModelGroup* joint_group, 
                                        const double* joint_group_variable_value){
            bool is_ik_solution_valid = true;
            double error;
            ROS_DEBUG("Validating new IK solution with %d invalid IK states", (int) invalid_states.size());
            for (auto invalid_ik = invalid_states.cbegin(); invalid_ik != invalid_states.cend(); invalid_ik++){
              error = 0;
              for( int i = 0; i < joint_group->getVariableCount(); i++){
                ROS_DEBUG(" q%d: %.5f %.5f ", i, joint_group_variable_value[i], (*invalid_ik)[i]);
                error += std::abs(joint_group_variable_value[i] - (*invalid_ik)[i]);
              }
              error /= 6;
              if( error < 0.35 ){ // IK solution found is one of the already inoperative configurations.
                is_ik_solution_valid = false;
                break;
              }
            }
            ROS_DEBUG_COND( !is_ik_solution_valid, "%dth th IK solution rejected", (int) invalid_states.size() + 1 );
            return is_ik_solution_valid;
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

          bool validateJointSpaceTargetMotion(MoveGroupInterface  *move_group,
                                              const robot_state::RobotState &start_state,
                                              const robot_state::RobotState &target_state,
                                              MoveGroupInterface::Plan *cartesian_motion_plan
                                              ){
            move_group->setStartState(start_state);
            move_group->setJointValueTarget(target_state);
            if (move_group->plan(*cartesian_motion_plan) != MoveItErrorCode::SUCCESS) {            
              return false;
            }
            return true;
          }

          void saveInvalidIK(const robot_state::RobotState &robot_state, 
                              const robot_state::JointModelGroup* jmb) {
            std::vector<int> joint_indices = jmb->getVariableIndexList(); 
            std::vector<double> invalid_group_ik;
            const double* variables = robot_state.getVariablePositions();
            for(int i = 0; i < joint_indices.size(); i++){
              invalid_group_ik.push_back( variables[joint_indices[i]] );
              ROS_DEBUG("Q%d: %.5f", joint_indices[i], variables[joint_indices[i]]);
            }
            invalid_states.push_back( invalid_group_ik );
          }

          float planCartesianMotionTask(MoveGroupInterface* move_group, 
                                                      std::vector<geometry_msgs::Pose> start_poses, 
                                                      std::vector<geometry_msgs::Pose> final_poses,
                                                      std::vector<MoveGroupInterface::Plan> &motion_plans,
                                                      const std::vector<std::string> allow_collision_objects,
                                                      uint max_configurations = 10) {
            ROS_INFO("Cartesian task planning for [%s]:\n Starting Poses: %d  \n Final Poses: %d \n Max IK solutions per pose: %d", 
                                                                      move_group->getName().c_str(),
                                                                      (int) start_poses.size(), 
                                                                      (int) final_poses.size(), 
                                                                      max_configurations);
            ros::Time start_time = ros::Time::now();
            bool task_plan_found = false;
            motion_plans.clear();
            invalid_states.clear();
            
        
            
           
            // Parameters for cartesian motion planning.
            moveit_msgs::RobotTrajectory trajectory, best_trajectory;
            const robot_state::RobotState current_state( *csda10f_mg.getCurrentState() );  // Planning start state to.
            robot_state::RobotState cartesian_start_state( *csda10f_mg.getCurrentState() );
            const robot_state::JointModelGroup *jmb = cartesian_start_state.getJointModelGroup(move_group->getName());
            double max_cartesian_motion = 0;
            const double jump_threshold = M_PI/2;
            const double eef_step = 0.04;  

            moveit_msgs::ApplyPlanningScene srv;
            moveit_msgs::PlanningScene after_grasp_ps_msg, grasp_ps_msg;
            planning_scene::PlanningScenePtr after_grasp_ps, grasp_ps = psm_->getPlanningScene();
            after_grasp_ps = grasp_ps->diff();
            after_grasp_ps->setName("After grasping...");
            grasp_ps->setName("Grasping...");
            // Allow collision during cartesian motion
            collision_detection::AllowedCollisionMatrix acm = after_grasp_ps->getAllowedCollisionMatrixNonConst();
            std::vector<std::string> eef_links_names = robot_model_->getEndEffector(jmb->getAttachedEndEffectorNames()[0])->getLinkModelNames();
            acm.setEntry( jmb->getLinkModelNames(), allow_collision_objects, true);
            acm.setEntry( eef_links_names, allow_collision_objects, true);
            moveit_msgs::AllowedCollisionMatrix acm_msg;
            acm.getMessage(acm_msg);
            after_grasp_ps_msg.allowed_collision_matrix = acm_msg;
            after_grasp_ps_msg.is_diff = true;
            after_grasp_ps_msg.name = "new ACM";
            // After manipulation planning scene
            acm.setEntry( jmb->getLinkModelNames(), allow_collision_objects, false);
            acm.setEntry( eef_links_names, allow_collision_objects, false);
            acm.getMessage(acm_msg);
            grasp_ps_msg.allowed_collision_matrix = acm_msg;
            grasp_ps_msg.is_diff = true;
            grasp_ps_msg.name = "Original";

            // ROS_WARN("post %d", acm.getEntry(jmb->getLinkModelNames()[0], eef_links_names[0]));
            // Start planning
            MoveGroupInterface::Plan cartesian_motion_plan, approach_plan, best_approach_plan;
                                            
            typedef std::vector<geometry_msgs::Pose>::iterator PoseIter;
            // Iterate through start poses.
            for (PoseIter start_pose_iter = start_poses.begin(); start_pose_iter != start_poses.end(); start_pose_iter++) {

              bool ik_solution_found = true;
              uint ik_configurations_count = 1;
              while (ik_solution_found) {
                auto ik_check_callback = boost::bind(&CSDA10F::validateIKSolution, this, _1, _2, _3);
                ik_solution_found = cartesian_start_state.setFromIK( jmb, *start_pose_iter, 10, 0.01, ik_check_callback );
                // Check if IK solution is found AND if a motion plan to reach it has been found 
                if (ik_solution_found && validateJointSpaceTargetMotion(move_group, current_state, cartesian_start_state, &approach_plan)) { 
                  // visual_tools->publishIKSolutions()
                  // Plan cartesian motion feasibility with current IK solution.
                  move_group->setStartState(cartesian_start_state);

                  srv.request.scene = after_grasp_ps_msg;
                  planning_scene_diff_client_.call(srv);
                  move_group->setGoalOrientationTolerance(30*M_PI/180);
                  double fraction_of_plan = 0;
                  // Iterate through final poses
                  for (PoseIter final_pose_iter = final_poses.begin(); final_pose_iter != final_poses.end(); final_pose_iter++) {
                    std::vector<geometry_msgs::Pose> waypoints = {*final_pose_iter};   
                    fraction_of_plan = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
                    if (fraction_of_plan > max_cartesian_motion) { // Best plan so far 
                      best_approach_plan = approach_plan;
                      best_trajectory = trajectory;
                      max_cartesian_motion = fraction_of_plan;
                      ROS_INFO("Pose %d (IK %d) allows for %.1f%% of cartesian motion", 
                        (int) std::distance(start_poses.begin(), start_pose_iter),
                        ik_configurations_count,
                        max_cartesian_motion * 100);
                    }
                    if (fraction_of_plan == 1)  // Current final goal is reachable through cartesian motion
                      break;
                  }
                  move_group->setGoalOrientationTolerance(5*M_PI/180);                  
                  srv.request.scene = grasp_ps_msg;
                  planning_scene_diff_client_.call(srv);

                  // Check cartesian plan result.
                  if (fraction_of_plan < 1) {
                    ROS_DEBUG_STREAM("Discarting start state (IK solution)");
                    // Save IK solution as invalid.
                    saveInvalidIK( cartesian_start_state, jmb );
                  } else {  // Full task motion plan found
                    task_plan_found = true;
                    break;
                  }
                } else if(ik_solution_found) {  // Solution is not reachable
                  // Save IK solution as invalid.
                  saveInvalidIK( cartesian_start_state, jmb );
                }
                // Check dropout condition
                if (ik_configurations_count >= max_configurations) 
                  break;
                ik_configurations_count++;                
              }
              if (task_plan_found) {
                max_cartesian_motion = 1.0;
                best_approach_plan = approach_plan;
                best_trajectory = trajectory;
                break;
              }
              invalid_states.clear();
            }  
            // Return original state to start state 
            move_group->setStartStateToCurrentState();
            invalid_states.clear();

            if (max_cartesian_motion == 0)
              return 0;
            // Return best possible plan 
            // Ensure motion plan end and start points are the same.
            moveit_msgs::RobotState state_msg;
            moveit::core::robotStateToRobotStateMsg( cartesian_start_state, state_msg);
            cartesian_motion_plan.trajectory_ = best_trajectory;
            cartesian_motion_plan.start_state_ = state_msg;
            cartesian_motion_plan.planning_time_ = 0.5;
            best_approach_plan.trajectory_.joint_trajectory.points.back().positions = best_trajectory.joint_trajectory.points[0].positions;
            // Save plan motions.
            motion_plans.push_back(best_approach_plan);
            motion_plans.push_back(cartesian_motion_plan);
            // visual_tools->publishTrajectoryPath(motion_plans[0].trajectory_, motion_plans[0].start_state_, false);
            // visual_tools->publishTrajectoryPath(motion_plans[1].trajectory_, motion_plans[0].start_state_, false);
            ros::Duration delta_t = ros::Time::now() - start_time;
            ROS_INFO("Task motion plan achieved in %.4f [s]" , delta_t.toSec());
            
            return max_cartesian_motion;                   
          }

          float planCartesianMotionTask(MoveGroupInterface* move_group, 
                                                      const geometry_msgs::Pose start_pose, 
                                                      const geometry_msgs::Pose final_pose,
                                                      std::vector<MoveGroupInterface::Plan> &motion_plans,
                                                      const std::vector<std::string> allow_collision_objects,
                                                      uint max_configurations = 10) {
            std::vector<geometry_msgs::Pose> start_poses{start_pose}, final_poses{final_pose};
            return planCartesianMotionTask(move_group, 
                                            start_poses, 
                                            final_poses, 
                                            motion_plans, 
                                            allow_collision_objects,
                                            max_configurations);
          }
        
        static trajectory_msgs::JointTrajectory getRightGripperPosture(float distance_between_fingers) {
          trajectory_msgs::JointTrajectory posture;
          posture.header.stamp = ros::Time::now();
          posture.joint_names = std::vector<std::string>{"right_gripper_finger_joint"};
          trajectory_msgs::JointTrajectoryPoint close_state;

          float joint_position = 0.7 - (0.7/0.140 * distance_between_fingers);
          joint_position = joint_position > 0.7 ? 0.7 : joint_position;
          joint_position = joint_position < 0.0 ? 0.0 : joint_position;

          close_state.positions.push_back(joint_position);
          close_state.velocities.push_back(0.1f);
          close_state.effort.push_back(0.0);
          close_state.time_from_start = ros::Duration(2.5);
          posture.points.push_back(close_state);
          return posture;
        }

        static trajectory_msgs::JointTrajectory getLeftGripperPosture(float distance_between_fingers) {
          trajectory_msgs::JointTrajectory posture;
          posture.header.stamp = ros::Time::now();
          posture.joint_names = std::vector<std::string>{"left_gripper_finger_joint"};
          trajectory_msgs::JointTrajectoryPoint close_state;
          
          float joint_position = 0.8 - (0.8/0.085 * distance_between_fingers);
          joint_position = joint_position > 0.8 ? 0.8 : joint_position;
          joint_position = joint_position < 0.0 ? 0.0 : joint_position;

          close_state.positions.push_back(joint_position);
          close_state.velocities.push_back(0.1f);
          close_state.effort.push_back(0.0);
          close_state.time_from_start = ros::Duration(2.5);
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