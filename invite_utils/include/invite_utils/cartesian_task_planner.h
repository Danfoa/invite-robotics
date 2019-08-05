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

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <industrial_msgs/RobotStatus.h>

namespace invite_utils {

typedef moveit::planning_interface::MoveItErrorCode MoveItErrorCode;
typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;


/**
 * Class holding fuctions to plan robot cartesian tasks, i.g. combination of robot motions required
 * to achieve a given cartesian path.  
 */
class CartesianTaskPlanner {
 
 private:
  // Set of most recent discarted robot configurations
  std::vector<std::vector<double>> invalid_states;
  ros::NodeHandlePtr nh_;

 public:
 
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  ros::ServiceClient planning_scene_diff_client_;


  CartesianTaskPlanner();

  /** Finds the most suitable robot configuration to perform the given cartesian `waypoints`
   * trajectory, by checking initially if a given robot configuration can be reached from the
   * currento robot state, and then evaluating how suitable is that particular configuration to
   * follow the cartesian trajectory.
   * 
   * @param move_group: Robot group to plan with
   * @param waypoints: Points defining the desire cartesian trajectory
   * @param motion_planes: Reference to a vector of motion plans that will be filled with the
   * approach and cartesian motion, result of the selection of the best robot start configuration
   * @param allow_collision_objecs: Vector with the IDs of collision objects that the gripper links
   * can touch during the cartesian motion
   * @param max_configurations: Maximum number of IK configuration to consider for a given target
   * pose. (usually the max number of IK solutions your robot has)
   *
   * @return percentage or cartesian motion achievable by the "best" robot start pose.
   */
  float planCartesianMotionTask(MoveGroupInterface* move_group,
                                std::vector<geometry_msgs::Pose> waypoints,
                                std::vector<MoveGroupInterface::Plan>& motion_plans,
                                const std::vector<std::string> allow_collision_objects,
                                const int max_configurations);
                                
  /** Return the set of discarted robot configuration of the last task planning.
   */
  std::vector<trajectory_msgs::JointTrajectoryPoint> getDiscartedRobotConfigurations();

  private:

    // Saves the `robot_state` joint values as invalid IK solution/robot configuration
    void saveInvalidIK(const robot_state::RobotState& robot_state,
                      const robot_state::JointModelGroup* jmb);

    // Check if a given set of joint values are not already in the discarted robot configurations array
    bool validateIKSolution(robot_state::RobotState* robot_state,
                            const robot_state::JointModelGroup* joint_group,
                            const double* joint_group_variable_value);
    
    /** Check if a robot configuration (`target_state`) can be reached from the a certain `start_state`
     *  @param `move_group`: Robot group to plan with
     *  @param `start_state`: Plan start robot configuration/IK.
     *  @param `target_state`: Plan final robot configuration/IK. 
     */
    bool validateJointSpaceTargetMotion(MoveGroupInterface* move_group,
                                        const robot_state::RobotState& start_state,
                                        const robot_state::RobotState& target_state,
                                        MoveGroupInterface::Plan* motion_plan);
};

}  // namespace invite_utils