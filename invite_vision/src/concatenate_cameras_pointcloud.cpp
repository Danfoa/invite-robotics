#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Syncronization dependencies 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_datatypes.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef sensor_msgs::PointCloud2 PointCloud2;

const int MSG_QUEUE = 8;

ros::Publisher pub;

// tf::Vector3 upper_cam_position(0.87251, 0.237802, 1.89592); 
// tf::Vector3 lower_cam_position(0.713277, 1.31721, 1.44293);
// tf::Quaternion upper_cam_orientation(-0.7070500191913, -0.695960729680754, 0.0898608112137424, 0.087429787325731);
// tf::Quaternion lower_cam_orientation(-0.00815435540059931, -0.83807389015755, 0.545479700303542, 0.00419018865331527);

void processCloud (const sensor_msgs::PointCloud2ConstPtr& input_upper, const sensor_msgs::PointCloud2ConstPtr& input_lower){
  // Create a container for the data.
  PointCloud2 output;
  std::string input_upper_field_list = pcl::getFieldsList( *input_upper ); 
  std::string input_lower_field_list = pcl::getFieldsList( *input_lower );
  ROS_WARN_STREAM("Receiving Upper pointcloud with height: " << input_upper->height << " and width: " << input_upper->width << " , with fields: " << input_upper_field_list);
  ROS_WARN_STREAM("Receiving Lower pointcloud with height: " << input_lower->height << " and width: " << input_lower->width << " , with fields: " << input_lower_field_list);

  // pcl::PointCloud<pcl::PointNormal>::Ptr joined_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>); 

  // Concatenate pointclouds ... TO-DO: Check if preprocessing independently is more efficient due to organized nature.!!!!
  ros::Time start_time = ros::Time::now();
  pcl::concatenatePointCloud( *input_upper, *input_lower, output);
  ros::Duration delta_t = ros::Time::now() - start_time;
  ROS_INFO_STREAM("Concatenation took: " << delta_t);
  ROS_WARN_STREAM_ONCE("Receiving Lower pointcloud with height: " << output.height << " and width: " << output.width << " , with fields: " << pcl::getFieldsList( output ));

  // Publish the data.
  pub.publish (output);
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "pre_processing");
  ros::NodeHandle nodeHandle;

  // Subscribe to both camera outputs with and approximate syncronize filter.
  message_filters::Subscriber<PointCloud2> pointCloudUpperSub(nodeHandle, "/n35_upper_camera/point_cloud", 1);
  message_filters::Subscriber<PointCloud2> pointCloudLowerSub(nodeHandle, "/n35_lower_camera/point_cloud", 1);
  
  typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> ApproximateTimePointCloudSync;
  
  Synchronizer<ApproximateTimePointCloudSync> sync ( ApproximateTimePointCloudSync(MSG_QUEUE) , pointCloudUpperSub, pointCloudLowerSub );
  sync.registerCallback(boost::bind(&processCloud, _1, _2));


  // Create a ROS publisher for the output point cloud
  pub = nodeHandle.advertise<sensor_msgs::PointCloud2> ("/joint_cameras_point_cloud", 1);

  // Spin
  ros::spin ();
}