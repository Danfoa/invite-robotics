#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Syncronization dependencies 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_datatypes.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef sensor_msgs::PointCloud2 PointCloud2;

const int MSG_SET_QUEUE = 1;

sensor_msgs::PointCloud2ConstPtr latest_upper_input;
sensor_msgs::PointCloud2ConstPtr latest_lower_input;
bool is_upper_cloud_ready;
bool is_lower_cloud_ready;
ros::Publisher pub;

void concatenateClouds (const sensor_msgs::PointCloud2ConstPtr& input_upper, 
                        const sensor_msgs::PointCloud2ConstPtr& input_lower) {
  is_upper_cloud_ready = is_lower_cloud_ready = false;
  // Create a container for the data.
  PointCloud2 output;

  // Concatenate pointclouds ... TO-DO: Check if preprocessing independently is more efficient due to organized nature.!!!!
  ros::Time start_time = ros::Time::now();
  pcl::concatenatePointCloud( *input_upper, *input_lower, output);
  ros::Duration delta_t = ros::Time::now() - start_time;
  ROS_INFO_STREAM("Concatenation took: " << delta_t);

  // Publish the data.
  pub.publish (output);
}

void processLowerCloud (const sensor_msgs::PointCloud2ConstPtr& cloud) { 
  latest_lower_input = cloud;
  is_lower_cloud_ready = true;
  // Debug information
  std::string field_list = pcl::getFieldsList( *cloud );
  ROS_INFO_STREAM("Receiving Lower pointcloud with height: " << cloud->height 
                                                             << " and width: " 
                                                             << cloud->width);
  if (is_lower_cloud_ready && is_upper_cloud_ready)
    concatenateClouds(latest_upper_input, latest_lower_input);
}

void processUpperCloud (const sensor_msgs::PointCloud2ConstPtr& cloud) { 
  latest_upper_input = cloud;
  is_upper_cloud_ready = true;
  // Debug information
  std::string field_list = pcl::getFieldsList( *cloud );
  ROS_INFO_STREAM("Receiving Upper pointcloud with height: " << cloud->height 
                                                             << " and width: " 
                                                             << cloud->width);
  if (is_lower_cloud_ready && is_upper_cloud_ready)
    concatenateClouds(latest_upper_input, latest_lower_input);
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "pre_processing");
  ros::NodeHandle nh;

  is_upper_cloud_ready = is_lower_cloud_ready = false;
  
  // Subscribe to both camera outputs with and approximate syncronize filter.
  // message_filters::Subscriber<PointCloud2> pointCloudUpperSub(nh, "/n35_upper_camera/point_cloud", 1);
  // message_filters::Subscriber<PointCloud2> pointCloudLowerSub(nh, "/n35_lower_camera/point_cloud", 1);
  
  // typedef message_filters::TimeSynchronizer<PointCloud2, PointCloud2> TimeSynchronizerCloudSync;
  ros::Subscriber sub_upper = nh.subscribe("/n35_upper_camera/point_cloud", 1, processUpperCloud);
  ros::Subscriber sub_lower = nh.subscribe("/n35_lower_camera/point_cloud", 1, processLowerCloud);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/joint_cameras_point_cloud", 1);

  // Spin
  ros::spin ();
  return 0;
}