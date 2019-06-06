#include <ros/ros.h>
#include <invite_utils/csda10f_interface.h>
#include <invite_utils/cartesian_task_planner.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rvt = rviz_visual_tools;


int main(int argc, char **argv){
    ros::init(argc, argv, "cartesian_task_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Instanciate robot interface, with max motion speed.
    float max_robot_speed = 0.5;   // From 0 to 1.0 meaning % of absolute speed
    invite_utils::CSDA10F csda10f(max_robot_speed);
    const robot_state::JointModelGroup *joint_model_group = csda10f.current_robot_state_ptr->getJointModelGroup("arm_left");
    csda10f.visual_tools->deleteAllMarkers();

    // Create instance of cartesian task planner.
    invite_utils::CartesianTaskPlanner task_planner;

    // Set up cartesian trajectory to follow --------------------------------------------------------------------------
    geometry_msgs::Pose initial_pose, final_pose;
    tf2::Quaternion q_orientation;
    // Set targer orientation as euler angles for ease of use
    //                ROLL-PITCH-YAW    [Radians]
    q_orientation.setRPY(0.0 , M_PI/2 , 0.0);
    tf2::convert(q_orientation, initial_pose.orientation);
    initial_pose.position.x = 0.8;
    initial_pose.position.y = 0.6;
    initial_pose.position.z = 1.2;

    final_pose = initial_pose;
    final_pose.position.x -= 0.25;
    final_pose.position.z += 0.80;
    final_pose.position.y += 0.1;
    q_orientation.setRPY(0.0 , M_PI/4  , M_PI/6);
    tf2::convert(q_orientation, final_pose.orientation);

    // Circular motion of the trajectory.
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

    // Plan cartesian motion takin into consideration the possible IK solutions of the robot --------------------------
    // Array that will contain two plans
    //      1. Approach plan to first pose of cartesian motion.
    //      2. Cartesian motion.
    std::vector<MoveGroupInterface::Plan> motion_plans; 

    csda10f.left_arm_mg.setStartStateToCurrentState();      // Optional
    csda10f.left_arm_mg.setGoalJointTolerance(15*M_PI/180); // Optional


    float motion_achievable = task_planner.planCartesianMotionTask(&(csda10f.left_arm_mg), 
                                                                waypoints,
                                                                motion_plans,
                                                                std::vector<std::string>{"camera_safe_zone.STL"},
                                                                20);
    ROS_INFO("%.2f%% of Cartesian task motion is possible", motion_achievable);
    
    // Visualize best plan 
    csda10f.visual_tools->publishIKSolutions(task_planner.getDiscartedRobotConfigurations(), joint_model_group, 0);
    csda10f.visual_tools->publishTrajectoryPath(motion_plans[0].trajectory_,  motion_plans[0].start_state_, false);        
    csda10f.visual_tools->publishTrajectoryPath(motion_plans[1].trajectory_, motion_plans[1].start_state_, false);        
    csda10f.visual_tools->trigger();

    // Perform approach and cartesian path motion.
    if (motion_achievable > 0.7) {
        csda10f.left_arm_mg.execute( motion_plans[0] ); // Approach motion.
        
        ros::Duration(0.1).sleep();

        csda10f.left_arm_mg.execute( motion_plans[1] ); // Cartesian motion.
    }

    ros::shutdown();
    return 0;
}