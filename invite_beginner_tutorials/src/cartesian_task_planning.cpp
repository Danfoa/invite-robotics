#include <ros/ros.h>
#include <invite_utils/csda10f_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualToolsPtr visual_tools;

int main(int argc, char **argv){
    ros::init(argc, argv, "cartesian_task_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/rviz_visual_tools"));

    invite_utils::CSDA10F csda10f(nh, 0.1);
    ros::Subscriber robot_state_sub = nh.subscribe<industrial_msgs::RobotStatus>("/robot_status", 
                                                                                    1,
                                                                                    &invite_utils::CSDA10F::updateRobotStatus,
                                                                                    &csda10f);
    csda10f.right_gripper->open();


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
    final_pose.position.z += 0.30;
    final_pose.position.y += 0.1;
    orientation.setRPY(0.0 , M_PI/2 , M_PI/6);

    visual_tools->deleteAllMarkers();
    visual_tools->publishAxisLabeled( initial_pose, "Initial", rvt::scales::MEDIUM);
    visual_tools->publishAxisLabeled( final_pose, "Final", rvt::scales::MEDIUM);
    visual_tools->trigger();
    std::vector<MoveGroupInterface::Plan> motion_plans; // Array containing the sequence of motion plans
    
    while( ros::ok() ){
        csda10f.left_arm_mg.setStartStateToCurrentState();
        bool is_motion_possible = csda10f.planCartesianMotionTask(&(csda10f.left_arm_mg), initial_pose, final_pose, motion_plans);

        if (is_motion_possible) {
            // visual_tools->publishTrajectoryPath(motion_plans[0].trajectory_, motion_plans[0].start_state_, false);
            visual_tools->trigger();
            // visual_tools->prompt("Perform motion approach");
            csda10f.left_arm_mg.execute( motion_plans[0] ); // Approach motion.
            // csda10f.right_gripper->close();
            visual_tools->trigger();
            // visual_tools->prompt("Perform motion approach");
            csda10f.left_arm_mg.execute( motion_plans[1] ); // Cartesian motion.
        }else{
            break;
        }
        ros::Duration(0.5).sleep();
    }
    return 0;
}