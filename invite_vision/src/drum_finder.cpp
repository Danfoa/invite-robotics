#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/sample_consensus/sac_model_cylinder.h>

#include <tf/transform_datatypes.h>

using namespace sensor_msgs;

typedef sensor_msgs::PointCloud2 PointCloud2;

ros::Publisher pub;

void processCloud (const sensor_msgs::PointCloud2ConstPtr& input){
  ros::Time start_time = ros::Time::now();
  
  // Create a container for the data.
  PointCloud2 output;

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::Normal> seg; 
  pcl::ExtractIndices<pcl::PointNormal> extract;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());

  pcl::fromROSMsg( *input, *cloud );
  pcl::fromROSMsg( *input, *cloud_normals );
  ROS_INFO_STREAM("cloud [" << (int) cloud->width << "," << (int) cloud->height <<"]");

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  Eigen::Vector3f z_axis(0,0,1);
  seg.setAxis( z_axis );
  seg.setMaxIterations (20000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.1, 0.4);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointNormal> ());
  extract.filter (*cloud_cylinder);
  // 

  ros::Duration delta_t = ros::Time::now() - start_time;
  ROS_INFO_STREAM("Finding the drum took: " << delta_t);
  pcl::toROSMsg( *cloud_cylinder, output);
  // Publish the data.
  pub.publish(output);
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "drum_finder");
  ros::NodeHandle nodeHandle;

  // Subscribe to input pointcloud topic 
  ros::Subscriber sub = nodeHandle.subscribe<sensor_msgs::PointCloud2> ("/joint_cameras_point_cloud/area_of_interest", 1, processCloud);
  
  pub = nodeHandle.advertise<PointCloud2> ("/joint_cameras_point_cloud/drum_inliers", 1);

  // Spin
  ros::spin ();
}