/*
 * This file is part of invite_utils package.
 *
 * To-Do: Add licence description
 *
 * File name: cartesian_motion_task_planner.h
 *
 * Description:
 *       Class to find the appropiate start configuration of a robot in order to fulfil a given
 *       cartesian path.
 *
 * Author:
 *        Daniel Felipe Ordonez Apraez - daniels.ordonez@gmail.com - OrdonezApraez@invite-research.com
 */

#include "invite_utils/cartesian_task_planner.h"
 
namespace invite_utils{

  CartesianTaskPlanner::CartesianTaskPlanner() { 

    nh_.reset(new ros::NodeHandle());

    psm_.reset( new planning_scene_monitor::PlanningSceneMonitor("robot_description") );
    // psm_->requestPlanningSceneState();
    psm_->publishDebugInformation(true);
    psm_->startSceneMonitor("/move_group/monitored_planning_scene");

    planning_scene_diff_client_ = nh_->serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client_.waitForExistence();

    invalid_states = {};
  }

  float CartesianTaskPlanner::planCartesianMotionTask(MoveGroupInterface* move_group,
                                std::vector<geometry_msgs::Pose> waypoints,
                                std::vector<MoveGroupInterface::Plan>& motion_plans,
                                const std::vector<std::string> allow_collision_objects,
                                const int max_configurations) {
    ROS_INFO(
      "Cartesian task planning for group %s and a trajectory with %d waypoints. Max IK solutions per pose: %d",
      move_group->getName().c_str(), (int)waypoints.size(), max_configurations);
    ros::Time start_time = ros::Time::now();
    bool task_plan_found = false;
    motion_plans.clear();
    invalid_states.clear();

    // Parameters for cartesian motion planning.
    moveit_msgs::RobotTrajectory trajectory, best_trajectory;
    const robot_state::RobotState current_state(*move_group->getCurrentState());  // Planning start state to.
    robot_state::RobotState cartesian_start_state(current_state);
    const robot_state::JointModelGroup* jmb =
        cartesian_start_state.getJointModelGroup(move_group->getName());

    const double jump_threshold = M_PI / 2;
    const double eef_step = 0.05;
    double max_cartesian_motion = 0;

    moveit_msgs::ApplyPlanningScene srv;
    moveit_msgs::PlanningScene after_grasp_ps_msg, grasp_ps_msg;
    planning_scene::PlanningScenePtr after_grasp_ps, grasp_ps = psm_->getPlanningScene();

    robot_model::RobotModelConstPtr robot_model = psm_->getRobotModel();

    after_grasp_ps = grasp_ps->diff();
    after_grasp_ps->setName("After grasping...");
    grasp_ps->setName("Grasping...");
    // Allow collision during cartesian motion
    collision_detection::AllowedCollisionMatrix acm =
        after_grasp_ps->getAllowedCollisionMatrixNonConst();
    std::vector<std::string> eef_links_names =
        robot_model->getEndEffector(jmb->getAttachedEndEffectorNames()[0])
            ->getLinkModelNames();
    acm.setEntry(jmb->getLinkModelNames(), allow_collision_objects, true);
    acm.setEntry(eef_links_names, allow_collision_objects, true);
    moveit_msgs::AllowedCollisionMatrix acm_msg;
    acm.getMessage(acm_msg);
    after_grasp_ps_msg.allowed_collision_matrix = acm_msg;
    after_grasp_ps_msg.is_diff = true;
    after_grasp_ps_msg.name = "new ACM";
    // After manipulation planning scene
    acm.setEntry(jmb->getLinkModelNames(), allow_collision_objects, false);
    acm.setEntry(eef_links_names, allow_collision_objects, false);
    acm.getMessage(acm_msg);
    grasp_ps_msg.allowed_collision_matrix = acm_msg;
    grasp_ps_msg.is_diff = true;
    grasp_ps_msg.name = "Original";

    MoveGroupInterface::Plan cartesian_motion_plan, approach_plan,
        best_approach_plan;

    typedef std::vector<geometry_msgs::Pose>::iterator PoseIter;
    // Iterate through start poses.

    geometry_msgs::Pose start_pose = waypoints.front();
    geometry_msgs::Pose final_pose = waypoints.back();

    bool ik_solution_found = true;
    uint ik_configurations_count = 1;
    while (ik_solution_found) {
      auto ik_check_callback =
          boost::bind(&CartesianTaskPlanner::validateIKSolution, this, _1, _2, _3);
      ik_solution_found = cartesian_start_state.setFromIK(
          jmb, start_pose, 10, 0.01, ik_check_callback);
      // Check if IK solution is found AND if a motion plan to reach it has been found
      if (ik_solution_found && validateJointSpaceTargetMotion(
                                    move_group, current_state,
                                    cartesian_start_state, &approach_plan)) {
        // visual_tools->publishIKSolutions()
        ROS_INFO("IK solution No. %d", ik_configurations_count);
        // Plan cartesian motion feasibility with current IK solution.
        move_group->setStartState(cartesian_start_state);

        srv.request.scene = after_grasp_ps_msg;
        planning_scene_diff_client_.call(srv);
        move_group->setGoalOrientationTolerance(30 * M_PI / 180);
        double fraction_of_plan = 0;
        // Iterate through final poses
        // std::vector<geometry_msgs::Pose> waypoints = {*final_pose_iter};
        fraction_of_plan = move_group->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory, true);
        if (fraction_of_plan > max_cartesian_motion) {  // Best plan so far
          best_approach_plan = approach_plan;
          best_trajectory = trajectory;
          max_cartesian_motion = fraction_of_plan;
         
        }
        ROS_INFO("IK solution No. %d allows for %.1f%% of cartesian motion",
                  ik_configurations_count, fraction_of_plan * 100);

        move_group->setGoalOrientationTolerance(5 * M_PI / 180);
        srv.request.scene = grasp_ps_msg;
        planning_scene_diff_client_.call(srv);

        // Check cartesian plan result.
        if (fraction_of_plan < 1) {
          ROS_DEBUG_STREAM("Discarting start state (IK solution)");
          // Save IK solution as invalid.
          saveInvalidIK(cartesian_start_state, jmb);
        } else {
          task_plan_found = true;
          break;
        }
      } else if (ik_solution_found) {  // Solution is not reachable
        // Save IK solution as invalid.
        ROS_INFO("IK Solution No. %d is not reachable from current robot state", ik_configurations_count);
        saveInvalidIK(cartesian_start_state, jmb);
      } else 
        ROS_ERROR("Trajectory start position is not reachable by the robot");

      // Check dropout condition
      if (ik_configurations_count >= max_configurations)
        break;
      ik_configurations_count++;
    }
    if (task_plan_found) {
      max_cartesian_motion = 1.0;
      best_approach_plan = approach_plan;
      best_trajectory = trajectory;
    }
    
    // Return original state to start state
    move_group->setStartStateToCurrentState();

    if (max_cartesian_motion == 0)
      return 0;
    // Return best possible plan
    // Ensure motion plan end and start points are the same.
    moveit_msgs::RobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(cartesian_start_state, state_msg);
    cartesian_motion_plan.trajectory_ = best_trajectory;
    cartesian_motion_plan.start_state_ = state_msg;
    cartesian_motion_plan.planning_time_ = 0.5;
    best_approach_plan.trajectory_.joint_trajectory.points.back().positions =
        best_trajectory.joint_trajectory.points[0].positions;
    // Save plan motions.
    motion_plans.push_back(best_approach_plan);
    motion_plans.push_back(cartesian_motion_plan);
    // visual_tools->publishTrajectoryPath(motion_plans[0].trajectory_,
    // motion_plans[0].start_state_, false);
    // visual_tools->publishTrajectoryPath(motion_plans[1].trajectory_,
    // motion_plans[0].start_state_, false);
    ros::Duration delta_t = ros::Time::now() - start_time;
    ROS_INFO("Task motion plan achieved in %.4f [s]", delta_t.toSec());

    return max_cartesian_motion;
  }
  
  std::vector<trajectory_msgs::JointTrajectoryPoint> CartesianTaskPlanner::getDiscartedRobotConfigurations(){ 
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_states;
    
    ROS_INFO("Publishing %d discarted start IK configurations.",(int) invalid_states.size());
    for (auto invalid_ik = invalid_states.cbegin();
            invalid_ik != invalid_states.cend(); invalid_ik++) {
      trajectory_msgs::JointTrajectoryPoint ik;
      ik.positions = *invalid_ik;
      ik_states.push_back(ik);
    }
    return ik_states;
  
  }

  void CartesianTaskPlanner::saveInvalidIK(const robot_state::RobotState& robot_state,
                    const robot_state::JointModelGroup* jmb) {
    std::vector<int> joint_indices = jmb->getVariableIndexList();
    std::vector<double> invalid_group_ik;
    const double* variables = robot_state.getVariablePositions();
    for (int i = 0; i < joint_indices.size(); i++) {
      invalid_group_ik.push_back(variables[joint_indices[i]]);
      ROS_DEBUG("Q%d: %.5f", joint_indices[i], variables[joint_indices[i]]);
    }
    invalid_states.push_back(invalid_group_ik);
  }

  bool CartesianTaskPlanner::validateIKSolution(robot_state::RobotState* robot_state,
                          const robot_state::JointModelGroup* joint_group,
                          const double* joint_group_variable_value) {
    bool is_ik_solution_valid = true;
    double error;
  
    ROS_DEBUG("Validating new IK solution with %d invalid IK states",
              (int)invalid_states.size());
    for (auto invalid_ik = invalid_states.cbegin();
          invalid_ik != invalid_states.cend(); invalid_ik++) {
      error = 0;
      for (int i = 0; i < joint_group->getVariableCount(); i++) {
        ROS_DEBUG(" q%d: %.5f %.5f ", i, joint_group_variable_value[i],
                  (*invalid_ik)[i]);
        error += std::abs(joint_group_variable_value[i] - (*invalid_ik)[i]);
      }
      error /= 6;
      if (error < 0.35) {  // IK solution found is one of the already
                            // inoperative configurations.
        is_ik_solution_valid = false;
        break;
      }
    }
    ROS_DEBUG_COND(!is_ik_solution_valid, "%dth th IK solution rejected",
                    (int)invalid_states.size() + 1);
    return is_ik_solution_valid;
  }
    
  bool CartesianTaskPlanner::validateJointSpaceTargetMotion(MoveGroupInterface* move_group,
                                      const robot_state::RobotState& start_state,
                                      const robot_state::RobotState& target_state,
                                      MoveGroupInterface::Plan* motion_plan) {
    move_group->setStartState(start_state);
    move_group->setJointValueTarget(target_state);
    if (move_group->plan(*motion_plan) != MoveItErrorCode::SUCCESS) {
      return false;
    }
    return true;
  }


}  // namespace invite_utils