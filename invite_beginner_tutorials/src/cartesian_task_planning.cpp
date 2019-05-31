#include <ros/ros.h>
#include <invite_utils/csda10f_interface.h>
#include <invite_utils/cartesian_task_planner.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;


int main(int argc, char **argv){
    ros::init(argc, argv, "cartesian_task_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    invite_utils::CSDA10F csda10f( 1);
    invite_utils::CartesianTaskPlanner task_planner;

    // csda10f.visual_tools->loadMarkerPub(true);

    // csda10f.right_gripper->open();


    geometry_msgs::Pose initial_pose, final_pose;
    tf::Quaternion orientation;
    // Set targer orientation as euler angles for ease of use
    //                ROLL-PITCH-YAW    [Radians]
    orientation.setRPY(0.0 , M_PI/2 , 0.0);
    initial_pose.orientation.x = orientation.x();
    initial_pose.orientation.y = orientation.y();
    initial_pose.orientation.z = orientation.z();
    initial_pose.orientation.w = orientation.w();
    initial_pose.position.x = 0.8;
    initial_pose.position.y = 0.6;
    initial_pose.position.z = 1.2;

    final_pose = initial_pose;
    final_pose.position.x -= 0.30;
    final_pose.position.z += 0.80;
    final_pose.position.y += 0.1;
    orientation.setRPY(0.0 , M_PI/4  , M_PI/6);
    final_pose.orientation.x = orientation.x();
    final_pose.orientation.y = orientation.y();
    final_pose.orientation.z = orientation.z();
    final_pose.orientation.w = orientation.w();

    csda10f.visual_tools->deleteAllMarkers();
    // csda10f.visual_tools->publishAxisLabeled( initial_pose, "Initial", rvt::scales::MEDIUM);
    // csda10f.visual_tools->publishAxisLabeled( final_pose, "Final", rvt::scales::MEDIUM);
    // csda10f.visual_tools->trigger();
    std::vector<MoveGroupInterface::Plan> motion_plans; // Array containing the sequence of motion plans
  
    const robot_state::JointModelGroup *joint_model_group = csda10f.current_robot_state_ptr->getJointModelGroup("arm_left");

    ros::Duration(0.5).sleep();

    // Plan a cartesian trajectory motion with different IK start states.

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose temp_pose = initial_pose;
    temp_pose.orientation = initial_pose.orientation;

    for (float theta = 0; theta < 2*M_PI; theta+= 10 * M_PI/180) {
        temp_pose.position.x = initial_pose.position.x + 0.15*std::sin(theta);
        temp_pose.position.y = initial_pose.position.y - 0.1*std::cos(theta);
        temp_pose.position.z = initial_pose.position.z + 0.2*std::sin(theta);
        waypoints.push_back(temp_pose);
    }

    waypoints.push_back(final_pose);

    csda10f.visual_tools->publishPath(waypoints, rvt::ORANGE, rvt::SMALL);
    csda10f.visual_tools->publishAxisLabeled( waypoints.front(), "Initial", rvt::scales::MEDIUM);
    csda10f.visual_tools->trigger();



    csda10f.left_arm_mg.setStartStateToCurrentState();

    csda10f.left_arm_mg.setGoalJointTolerance(15*M_PI/180);

    float motion_achievable = task_planner.planCartesianMotionTask(&(csda10f.left_arm_mg), 
                                                                waypoints,
                                                                motion_plans,
                                                                std::vector<std::string>{"camera_safe_zone.STL"},
                                                                20);
    ROS_INFO("%.2f%% of Motion is possible", motion_achievable);
    if (motion_achievable > 0.5) {
        // visual_tools->publishTrajectoryPath(motion_plans[0].trajectory_, motion_plans[0].start_state_, false);
        trajectory_msgs::JointTrajectoryPoint ik;
        std::vector<trajectory_msgs::JointTrajectoryPoint> iks;
        ik = motion_plans[1].trajectory_.joint_trajectory.points.front();
        iks.push_back(ik);
        csda10f.visual_tools->publishRobotState(ik, joint_model_group, rvt::colors::GREEN);
        csda10f.visual_tools->trigger();
        // visual_tools->prompt("Perform motion approach");
        csda10f.left_arm_mg.execute( motion_plans[0] ); // Approach motion.
        // visual_tools->publishTrajectoryPath(motion_plans[1].trajectory_, motion_plans[1].start_state_, false);
        ik = motion_plans[1].trajectory_.joint_trajectory.points.back();
        // visual_tools->publishRobotState(ik, joint_model_group, rvt::colors::BLUE);
        // csda10f.right_gripper->close();
        // visual_tools->trigger();
        ros::Duration(0.1).sleep();
        // visual_tools->prompt("Perform motion approach");
        csda10f.left_arm_mg.execute( motion_plans[1] ); // Cartesian motion.
    }
    ros::shutdown();
    return 0;
}