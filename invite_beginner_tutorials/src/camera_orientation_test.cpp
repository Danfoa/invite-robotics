

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>
#include <tf/transform_datatypes.h>
// #include <tf/Scalar.h>

// typedef tf tf2;

int main(int argc, char **argv){
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const double Z_OFFSET = 1.2;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  geometry_msgs::Pose upper_camera_pose;
  geometry_msgs::Pose lower_camera_pose;
  // Create a quaternion instance as it is required by 'geometry_msgs::Pose'
  tf::Quaternion upper_cam_orientation;
  upper_cam_orientation.setW( 0.087429787325731);
  upper_cam_orientation.setX(-0.7070500191913);
  upper_cam_orientation.setY(-0.695960729680754);
  upper_cam_orientation.setZ( 0.0898608112137424);
  // Set targer upper_cam_orientation as euler angles for ease of use


  // Show frames
  upper_camera_pose.orientation.x = upper_cam_orientation.x();
  upper_camera_pose.orientation.y = upper_cam_orientation.y();
  upper_camera_pose.orientation.z = upper_cam_orientation.z();
  upper_camera_pose.orientation.w = upper_cam_orientation.w();
  upper_camera_pose.position.x = 0.77250982549;     //[meters]
  upper_camera_pose.position.y = 0.237802138;     //[meters]
  upper_camera_pose.position.z = 0.6959212 + Z_OFFSET;      //[meters]
  
  visual_tools.publishAxisLabeled(upper_camera_pose, "upper_cam");
  // Print roll pitch yaw
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 rotation_matrix( upper_cam_orientation );
  rotation_matrix.getRPY( roll, pitch, yaw);
  
  ROS_INFO_STREAM("Upper cammera rpy: " << roll << " " << pitch << " " << yaw); 
  ROS_INFO_STREAM("Upper cammera xyz: " << upper_camera_pose.position.x << " " << upper_camera_pose.position.y << " " << upper_camera_pose.position.z); 
  visual_tools.trigger();
  
  // ===================================================================================================
  
  tf::Quaternion lower_cam_orientation;
  lower_cam_orientation.setW( 0.00419018865331527);
  lower_cam_orientation.setX(-0.00815435540059931);
  lower_cam_orientation.setY(-0.83807389015755);
  lower_cam_orientation.setZ( 0.545479700303542);

  // Show frames
  lower_camera_pose.orientation.x = lower_cam_orientation.x();
  lower_camera_pose.orientation.y = lower_cam_orientation.y();
  lower_camera_pose.orientation.z = lower_cam_orientation.z();
  lower_camera_pose.orientation.w = lower_cam_orientation.w();
  lower_camera_pose.position.x = 0.61327675322;     //[meters]
  lower_camera_pose.position.y = 1.317213161;     //[meters]
  lower_camera_pose.position.z = 0.24292913395 + Z_OFFSET;      //[meters]
  
  visual_tools.publishAxisLabeled(lower_camera_pose, "lower_cam");
  // Print roll pitch yaw
  // tfScalar yaw, pitch, roll;
  tf::Matrix3x3 rotation_matrix2( lower_cam_orientation );
  rotation_matrix2.getRPY( roll, pitch, yaw);

  ROS_INFO_STREAM("Lower cammera rpy: " << roll << " " << pitch << " " << yaw ); 
  ROS_INFO_STREAM("Lower cammera xyz: " << lower_camera_pose.position.x << " " << lower_camera_pose.position.y << " " << lower_camera_pose.position.z); 
  visual_tools.trigger();

  
  ros::shutdown();
  return 0;
}