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
#include <pcl/sample_consensus/sac_model_cylinder.h>

#include <tf/transform_datatypes.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <dynamic_reconfigure/server.h>
#include <invite_vision/DrumFinderConfig.h>


#include <math.h>
// #include <stdio.h>
// #include <stdlib.h>
#include <algorithm> 
#include <iterator>
#include <gsl/gsl_histogram.h>

using namespace sensor_msgs;


// Types 
typedef sensor_msgs::PointCloud2 PointCloud2;

// Forward declarations 
double getDrumHeight( pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointIndices::Ptr);
void publishDrumCollisionObject(  Eigen::VectorXf& );
Eigen::VectorXf findCylinderCoefficients(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr, pcl::PointIndices::Ptr);
void findCylinderInliers( pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointIndices::Ptr, Eigen::VectorXf&);
void parameter_update(invite_vision::DrumFinderConfig&, uint32_t);

ros::Publisher pub;

// Dynamic parameters 
double eps_angle;
double normals_weight;
double max_iterations;
double distance_threshold;
double z_normal_weight;

Eigen::VectorXf drum_params; /* Structure: 
                                point_on_axis.x : the X coordinate of a point located on the cylinder axis
                                point_on_axis.y : the Y coordinate of a point located on the cylinder axis
                                point_on_axis.z : the Z coordinate of a point located on the cylinder axis
                                axis_direction.x : the X coordinate of the cylinder's axis direction
                                axis_direction.y : the Y coordinate of the cylinder's axis direction
                                axis_direction.z : the Z coordinate of the cylinder's axis direction
                                radius : the cylinder's radius
                                height : highest z coordinate of the cylinder
                            */
bool drum_found;

void processCloud (const sensor_msgs::PointCloud2ConstPtr& input){
  // Create a container for the data.
  PointCloud2 output;

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::PointIndices::Ptr inliers_cylinder( new pcl::PointIndices );
  pcl::ExtractIndices<pcl::PointNormal> extract;

  pcl::fromROSMsg( *input, *cloud );
  pcl::fromROSMsg( *input, *cloud_normals );
  ROS_DEBUG_STREAM("input cloud size [" << (int) cloud->width << "," << (int) cloud->height <<"]");

  if( !drum_found ){
    Eigen::VectorXf observed_drum_params = findCylinderCoefficients( cloud , cloud_normals, inliers_cylinder);
    // Check if the drum found is reliable.
    if( inliers_cylinder->indices.size() < 100)
      ROS_ERROR_STREAM("Drum not found! " << inliers_cylinder->indices.size()<< " inliers");
    else{
      observed_drum_params[7] = getDrumHeight( cloud, inliers_cylinder );
      // // TO-DO: perform filtering or learning rate
      if( drum_params.size() != 8 )    // On first reading use raw observation.
        drum_params = observed_drum_params;
      else{                           // Modify the state estimate using the previous estimate and the current observation 
        Eigen::VectorXf prev_params, error = observed_drum_params - drum_params;
        float error_norm = error.cwiseQuotient(drum_params).norm() / 8;
        prev_params = drum_params;
        drum_params = drum_params + (1 - error_norm/(1 + error_norm)) * error;
        ROS_INFO("Drum parameters error %.5f", error_norm);
        if( error_norm < 0.1){
          drum_found = true;
          ROS_WARN("Drum parameters fine tunning done: Height: %.2f Radius: %.2f X: %.2f Y: %.2f", drum_params[7], drum_params[6], drum_params[0], drum_params[1]);
        }
      }
      
    }
  }else // Use found parameters to eliminate drum points from cloud and label bag points  
    findCylinderInliers( cloud, inliers_cylinder, drum_params);

    // Write the drum inliers to disk
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative ( drum_found );
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointNormal> ());
  extract.filter (*cloud_cylinder);
  pcl::toROSMsg( *cloud_cylinder, output);
  // Publish the data.
  pub.publish(output);
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "drum_finder");
  ros::NodeHandle nodeHandle;

  // Set dynamic parameters configuration
  dynamic_reconfigure::Server<invite_vision::DrumFinderConfig> server;
  dynamic_reconfigure::Server<invite_vision::DrumFinderConfig>::CallbackType callback;
  callback = boost::bind(&parameter_update, _1, _2);
  server.setCallback(callback);

  // Subscribe to input pointcloud topic 
  ros::Subscriber sub = nodeHandle.subscribe<sensor_msgs::PointCloud2> ("/joint_cameras_point_cloud/area_of_interest", 1, processCloud);
  
  pub = nodeHandle.advertise<PointCloud2> ("/joint_cameras_point_cloud/bag_inliers", 1);

  // Spin
  ros::spin ();
}

Eigen::VectorXf findCylinderCoefficients( pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_cylinder ){
  ros::Time start_time = ros::Time::now();
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

  pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::Normal> seg; 
  
  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (normals_weight);
  Eigen::Vector3f z_axis(0,0,1);
  seg.setAxis( z_axis );
  seg.setEpsAngle( eps_angle * M_PI/180 );
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold (distance_threshold);
  seg.setRadiusLimits (0.1, 0.4);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  ros::Duration delta_t = ros::Time::now() - start_time;
  Eigen::VectorXf drum_params(8);
  for( int i = 0; i < 8; i++ )
    drum_params[i] = coefficients_cylinder->values[i];

  ROS_INFO_STREAM("Finding the drum took: " << delta_t);
  return drum_params;
}

void findCylinderInliers( pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointIndices::Ptr inliers_cylinder, Eigen::VectorXf& drum_params ){
  ros::Time start_time = ros::Time::now();
  // Clear inliers vector
  inliers_cylinder->indices.clear();

  double drum_top = drum_params[7];
  for(pcl::PointCloud<pcl::PointNormal>::iterator point = cloud->begin(); point!= cloud->end(); point++){
    int index = std::distance(cloud->begin(), point); 
    if( point->z < drum_top * 1.15 ){  // Points below drum top
      double distance = std::sqrt( std::pow(point->x - drum_params[0], 2) + std::pow(point->y - drum_params[1], 2) );      // Get distance to drum axis squared 
      if( std::abs(distance - drum_params[6]) < distance_threshold )                                                      // Check if point is inlier
        inliers_cylinder->indices.push_back(index);
    }
  }
  // Obtain the cylinder inliers and coefficients
  ros::Duration delta_t = ros::Time::now() - start_time;
  ROS_INFO_STREAM("Finding the drum inliers took: " << delta_t);
  ROS_INFO("%d Drum Inliers found: ", (int) inliers_cylinder->indices.size());
}


double getDrumHeight( pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointIndices::Ptr inliers ){
  ROS_INFO("Finding drum top");
  const int MIN_EXPECTED_DRUM_HEIGHT = 0.6; // [m]
  const int MAX_EXPECTED_DRUM_HEIGHT = 1.1; // [m]
  const int NUM_BINS = 22;

  int num_inliers = inliers->indices.size();

  // Calculate min/max Z and Normal Z value 
  double min_normal_z = cloud->points[0].normal_z;
  double max_normal_z = cloud->points[0].normal_z;
  for(int i = 0; i < num_inliers; i++){
    double tmp_point_normal = cloud->points[ inliers->indices[i] ].normal_z;          // Access only drum inliers .
    double tmp_point_z = cloud->points[ inliers->indices[i] ].z;                      // Access only drum inliers .
    min_normal_z = min_normal_z > tmp_point_normal?  tmp_point_normal: min_normal_z;
    max_normal_z = max_normal_z < tmp_point_normal?  tmp_point_normal: max_normal_z;
  }
  // Calculate the maximum normal_z threshold   
  min_normal_z = max_normal_z - std::abs(max_normal_z - min_normal_z)*z_normal_weight;

  // Initialize histogram with a range from the lowest drum point to the highest 
  gsl_histogram *z_histogram = gsl_histogram_alloc( NUM_BINS );         // Array holding Z coordinates of inliers 
  gsl_histogram_set_ranges_uniform(z_histogram, MIN_EXPECTED_DRUM_HEIGHT, MAX_EXPECTED_DRUM_HEIGHT);

  // Fill histogram with only the points with the lowest 10% normal Z
  // std::vector<int> drum_top_indices; 
  for(int i = 0; i < num_inliers; i++)
    if( cloud->points[ inliers->indices[i] ].normal_z > min_normal_z )        // Access only drum inliers .
      gsl_histogram_increment( z_histogram, cloud->points[inliers->indices[i]].z );
      // drum_top_indices.push_back(inliers->indices[i]);
    
  ROS_INFO("Drum inliers with low normal Z values: %d", (int) gsl_histogram_sum(z_histogram) );

  double drum_top_bin = gsl_histogram_max_bin( z_histogram );
  double lower_bin_range, upper_bin_range;
  gsl_histogram_get_range( z_histogram, drum_top_bin, &lower_bin_range , &upper_bin_range);
  // ROS_WARN_STREAM("Max, Min z coordinates: " << max_normal_z << " -- " << min_normal_z << " - Drum top: " << lower_bin_range << " / " << upper_bin_range);
  // gsl_histogram_fprintf (stderr, z_histogram, "%g", "%g");
  // inliers->indices.swap( drum_top_indices );

  // Remove false drum inliers 
  for(int i = 0; i < num_inliers; i++)
    if( cloud->points[ inliers->indices[i] ].z > upper_bin_range ){        // Cylinder points above drum lid are not inliers.
      inliers->indices[i] = inliers->indices.back();
      inliers->indices.pop_back();
  }
  
  ROS_INFO("Drum top %.3f", upper_bin_range);
  return upper_bin_range;
}

void publishDrumCollisionObject( Eigen::VectorXf &drum_params){
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "/base_link";
  collision_object.header.stamp = ros::Time::now();
  // The id of the object is used to identify it.
  collision_object.id = "drum";

  // Define a cylinder to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  /* Setting height of cylinder. */
  primitive.dimensions[0] = drum_params[7];
  /* Setting radius of cylinder. */
  primitive.dimensions[1] = drum_params[6];

  geometry_msgs::Pose drum_pose;
  tf::Quaternion orientation;
  orientation.setRPY(0.0 , 0.0 , 0.0);        // Asume Drum axis is the robot Z axis 
  drum_pose.orientation.x = orientation.x();
  drum_pose.orientation.y = orientation.y();
  drum_pose.orientation.z = orientation.z();
  drum_pose.orientation.w = orientation.w();

  drum_pose.position.x = drum_params[0];
  drum_pose.position.y = drum_params[1];
  drum_pose.position.z = drum_params[7] / 2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(drum_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO("Applying collision cylinder shape");
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void parameter_update(invite_vision::DrumFinderConfig &config, uint32_t level) {
  ROS_INFO("Updating parameters \nMax iterations: %d\nEps Angle: %f\nNormals Weight: %f\nDistance Theshold: %f\nz_normal_weight: %f", config.max_iterations, config.eps_angle, config.normals_weight, config.distance_threshold, config.z_normal_weight);
  max_iterations = config.max_iterations;
  eps_angle = config.eps_angle;
  normals_weight = config.normals_weight;
  distance_threshold = config.distance_threshold;
  z_normal_weight = config.z_normal_weight;
}
