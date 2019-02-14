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
#include <moveit/robot_state/conversions.h>

#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_control/robotiq_gripper_client.h>

#include <industrial_msgs/RobotStatus.h>

#include <sound_play/SoundRequest.h>


typedef moveit::planning_interface::MoveItErrorCode MoveItErrorCode;
typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;

typedef robotiq_2f_gripper_control::RobotiqActionClient RobotiqActionClient;

namespace invite_utils{

    typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperAction CommandRobotiqGripperAction;

    class CSDA10F{
        private:        
          float max_velocity_scaling_factor;  
          float joint_goal_orientation_tolerance = 1e-4;
          // Publisher to Robot sound generator topic
          ros::Publisher sound_pub;
          // Boolean indicating whether robot should speak or not.
          bool talkative = false; 

          moveit::planning_interface::MoveGroupInterface::Plan cartesian_motion_plan;
          MoveItErrorCode error_code;

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

          CSDA10F(float max_speed = 0.1) : csda10f_mg("csda10f"), 
                      arms_mg("arms"),
                      torso_mg("torso"),
                      left_arm_mg("arm_left"),
                      left_arm_with_torso_mg("arm_left_with_torso"),
                      right_arm_mg("arm_right"),
                      right_arm_with_torso_mg("arm_right_with_torso"){
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
            ROS_WARN("Robot interface ready for motion at %.1f%%", max_velocity_scaling_factor * 100);
          }

          CSDA10F(ros::NodeHandle nh, float max_speed = 0.5): CSDA10F( max_speed ){
            this->talkative = true;
            sound_pub = nh.advertise<sound_play::SoundRequest>("/robotsound", 3);

          }

          bool goHome(std::string home_pose_name = "home") {
            if (talkative)
              speak("Going to " + home_pose_name + " position");
            // Take left arm home'
            csda10f_mg.setNamedTarget(home_pose_name);
            error_code = csda10f_mg.move();
            // To-Do: work arround while multi-group goal automatic abortion bug is fixed
            ros::Rate rate(5);
            while (ros::ok() && this->status.in_motion.val == true){
              rate.sleep();
              ROS_INFO("Waiting...");
            }
            if (error_code != MoveItErrorCode::SUCCESS){
              ROS_ERROR("Motion to position (%s) impossible, error code: (%d) %s", 
                          home_pose_name.c_str(), 
                          (int)error_code.val,
                          getErrorMsg(error_code).c_str()
                          );
              return false;
            }
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
            csda10f_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            arms_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            left_arm_with_torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            right_arm_with_torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            left_arm_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            right_arm_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
            torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
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
              if( error < 0.17 ){ // IK solution found is one of the already inoperative configurations.
                is_ik_solution_valid = false;
                break;
              }
            }
            ROS_WARN_COND( !is_ik_solution_valid, "%dth th IK solution rejected", (int) invalid_states.size() + 1 );
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
              if( new_status->in_error.val == industrial_msgs::TriState::ON )
                ROS_ERROR("Robot error: alarm % is active", new_status->error_code);
              if( new_status->in_motion.val == false && this->status.in_motion.val == true){
                current_robot_state_ptr = csda10f_mg.getCurrentState();
                ROS_INFO_STREAM("-- Robot State: Robot has just stopped its motion " << current_robot_state_ptr );
              }
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
                              const robot_state::JointModelGroup* joint_model_group) {
            std::vector<int> joint_indices = joint_model_group->getVariableIndexList(); 
            std::vector<double> invalid_group_ik;
            ROS_INFO("Save IK solution as invalid.");
            const double* variables = robot_state.getVariablePositions();
            for(int i = 0; i < joint_indices.size(); i++){
              invalid_group_ik.push_back( variables[joint_indices[i]] );
              ROS_DEBUG("Q%d: %.5f", joint_indices[i], variables[joint_indices[i]]);
            }
            invalid_states.push_back( invalid_group_ik );
          }

          bool planCartesianMotionTask(MoveGroupInterface* move_group, 
                                                      const geometry_msgs::Pose initial_pose, 
                                                      const geometry_msgs::Pose final_pose,
                                                      std::vector<MoveGroupInterface::Plan> &motion_plans,
                                                      uint max_configurations = 25) {
            ROS_INFO("Starting cartesian task planning for move group [%s]", move_group->getName().c_str());
            bool task_plan_found = false;
            motion_plans.clear();
            invalid_states.clear();

            // Parameters for cartesian motion planning.
            moveit_msgs::RobotTrajectory trajectory;
            // Allow for a lot of freedom between intermetiade poinvalid_states
            const double jump_threshold = M_PI/2;
            const double eef_step = 0.02;  
                                          
            MoveGroupInterface::Plan cartesian_motion_plan, approach_plan;

            const robot_state::RobotState current_state( *csda10f_mg.getCurrentState() );  // Planning start state to.
            robot_state::RobotState cartesian_start_state( current_state );
            const robot_state::JointModelGroup *joint_model_group = cartesian_start_state.getJointModelGroup(move_group->getName());
            // Define cartesian motion waypoints.
            std::vector<geometry_msgs::Pose> waypoints = {final_pose};

            bool ik_solution_found = true;
            uint ik_configurations_count = 0;
            while (ik_solution_found) {
              auto ik_check_callback = boost::bind(&CSDA10F::validateIKSolution, this, _1, _2, _3);
              ik_solution_found = cartesian_start_state.setFromIK( joint_model_group, initial_pose, 10, 0.05, ik_check_callback );
              ik_configurations_count++;
              // Check if IK solution is found AND if a motion plan to reach it has been found 
              if (ik_solution_found && validateJointSpaceTargetMotion(move_group, current_state, cartesian_start_state, &approach_plan)) { 
                // Plan cartesian motion feasibility with current IK solution.
                move_group->setStartState(cartesian_start_state);
                double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
                // Check cartesian plan result.
                if (fraction < 1) {
                  ROS_ERROR_STREAM("Motion planning failed, " << (fraction * 100) << " % of the movement is achievable");
                  // Save IK solution as invalid.
                  saveInvalidIK( cartesian_start_state, joint_model_group );
                } else {  // Task motion plan was found
                  task_plan_found = true;
                  break;
                }
              } else if(ik_solution_found) {  // Solution is not reachable
                // Save IK solution as invalid.
                saveInvalidIK( cartesian_start_state, joint_model_group );
              }
              // Check dropout condition
              if (ik_configurations_count > max_configurations) 
                break;
            }
            // Return original state to start state 
            move_group->setStartStateToCurrentState(); 

            if (task_plan_found) {
              // Ensure motion plan end and start points are the same.
              moveit_msgs::RobotState state_msg;
              moveit::core::robotStateToRobotStateMsg( cartesian_start_state, state_msg);
              cartesian_motion_plan.trajectory_ = trajectory;
              cartesian_motion_plan.start_state_ = state_msg;
              cartesian_motion_plan.planning_time_ = 0.5;
              approach_plan.trajectory_.joint_trajectory.points.back().positions = trajectory.joint_trajectory.points[0].positions;
              // Save plan motions.
              motion_plans.push_back(approach_plan);
              motion_plans.push_back(cartesian_motion_plan);
              ROS_INFO("-- Task motion plan achieved, storing %d trajectory plans", (int) motion_plans.size() );
            }else{
              ROS_ERROR("Task motion plan impossible");
            }

            waypoints.clear();
            invalid_states.clear();

            return task_plan_found;
          }

        private:

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