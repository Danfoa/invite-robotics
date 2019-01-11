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

// Planning scene and objects 
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <dynamic_reconfigure/server.h>
#include <invite_vision/DrumFinderConfig.h>
#include <invite_vision/DrumParams.h>

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
void findBagInliers( pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PointIndices::Ptr, Eigen::VectorXf&);
void parameter_update(invite_vision::DrumFinderConfig&, uint32_t);

ros::Publisher pub;
ros::Publisher drum_params_pub;

// Dynamic parameters 
double eps_angle;
double normals_weight;
double max_iterations;
double distance_threshold;
double z_normal_weight;

Eigen::VectorXf drum_params; /* Structure: 
                                0: point_on_axis.x : the X coordinate of a point located on the cylinder axis
                                1: point_on_axis.y : the Y coordinate of a point located on the cylinder axis
                                2: point_on_axis.z : the Z coordinate of a point located on the cylinder axis
                                3: axis_direction.x : the X coordinate of the cylinder's axis direction
                                4: axis_direction.y : the Y coordinate of the cylinder's axis direction
                                5: axis_direction.z : the Z coordinate of the cylinder's axis direction
                                6: radius : the cylinder's radius
                                7: height : highest z coordinate of the cylinder
                            */
bool drum_found;

void processCloud (const sensor_msgs::PointCloud2ConstPtr& input){
  // Create a container for the data.
  PointCloud2 output;

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
  pcl::ExtractIndices<pcl::PointNormal> extract;

  pcl::fromROSMsg( *input, *cloud );
  pcl::fromROSMsg( *input, *cloud_normals );
  ROS_DEBUG_STREAM("input cloud size [" << (int) cloud->width << "," << (int) cloud->height <<"]");

  if( !drum_found ){
    Eigen::VectorXf observed_drum_params = findCylinderCoefficients( cloud , cloud_normals, inliers);
    // Check if the drum found is reliable.
    int i = 0;
    while( inliers->indices.size() < 100 &&  i < 4){
      observed_drum_params = findCylinderCoefficients(cloud, cloud_normals, inliers);
      ROS_WARN_STREAM("Reinitializing search");
      i++;
    }
    if ( inliers->indices.size() < 100 )
      ROS_ERROR_STREAM("Drum not found! " << inliers->indices.size()<< " inliers");
    else{
      observed_drum_params[7] = getDrumHeight( cloud, inliers );
      // // TO-DO: perform filtering or learning rate
      if( drum_params.size() != 8 )    // On first reading use raw observation.
        drum_params = observed_drum_params;
      else{                           // Modify the state estimate using the previous estimate and the current observation 
                                                                                  // Do not consider point in Z nor Z rotation of the parameters
        Eigen::VectorXf prev_params, error = (observed_drum_params - drum_params).cwiseProduct((Eigen::VectorXf(8) << 1,1,0,0,0,0, 1, 1).finished());
        float relative_error = (error.cwiseQuotient(drum_params).cwiseAbs()).sum() / 8.0; 
        prev_params = drum_params;
        drum_params = drum_params + (1 - relative_error/(1 + relative_error)) * error;
        ROS_INFO("Drum parameters relative error %.2f [%%]", relative_error*100);
        ROS_DEBUG_STREAM("Prev: \n" << drum_params << " New: \n" << observed_drum_params << " Error: \n" << error.cwiseQuotient(drum_params).cwiseAbs());
        if( relative_error < 0.01 ){
          drum_found = true;
          ROS_WARN("Drum parameters fine tunning done: Height: %.3f Radius: %.4f X: %.4f Y: %.4f", drum_params[7], drum_params[6], drum_params[0], drum_params[1]);
        }
      }
      publishDrumCollisionObject(drum_params);
    }
  }else // Use found parameters to eliminate drum points from cloud and label bag points  
    findBagInliers( cloud, inliers, drum_params);

  // Write the drum/bag inliers to disk
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative ( false );
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
  pub = nodeHandle.advertise<invite_vision::DrumParams> ("/invite/drum_params", 1);

  // Spin
  ros::spin ();
}

Eigen::VectorXf findCylinderCoefficients( pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers ){
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
  seg.segment(*inliers, *coefficients_cylinder);
  ros::Duration delta_t = ros::Time::now() - start_time;
  Eigen::VectorXf drum_params(8);
  for( int i = 0; i < 8; i++ )
    drum_params[i] = coefficients_cylinder->values[i];

  ROS_INFO_STREAM("Locating the a drum with " << (int) inliers->indices.size() << " inliers took: " << delta_t << "[s]");
  // ROS_INFO_STREAM("Finding the drum took: " << delta_t);
  return drum_params;
}

void findBagInliers( pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointIndices::Ptr inliers, Eigen::VectorXf& drum_params ){
  ros::Time start_time = ros::Time::now();
  // Clear inliers vector
  inliers->indices.clear();

  double drum_top = drum_params[7];
  for(pcl::PointCloud<pcl::PointNormal>::iterator point = cloud->begin(); point!= cloud->end(); point++){
    int index = std::distance(cloud->begin(), point); 
    if( point->z < drum_top + 0.02 ){  // Points below drum top
      double distance = std::sqrt( std::pow(point->x - drum_params[0], 2) + std::pow(point->y - drum_params[1], 2) );      // Get distance to drum axis squared 
      if( distance < (drum_params[6] - 0.02) )                                                      // Check if point is inlier
        inliers->indices.push_back(index);
    }else{  // Points below drum top
      double distance = std::sqrt( std::pow(point->x - drum_params[0], 2) + std::pow(point->y - drum_params[1], 2) );      // Get distance to drum axis squared 
      if( distance < (2* drum_params[6]) )                                                      // Check if point is inlier
        inliers->indices.push_back(index);
    }
  }
  // Obtain the cylinder inliers and coefficients
  ros::Duration delta_t = ros::Time::now() - start_time;
  std::cerr << "\r" << "[ INFO] Finding " << (int) inliers->indices.size() << " drum inliers took: " << delta_t << "[s]";
}


double getDrumHeight( pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointIndices::Ptr inliers ){
  ROS_DEBUG("Finding drum top");
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
    
  ROS_DEBUG("Drum inliers with low normal Z values: %d", (int) gsl_histogram_sum(z_histogram) );

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
  
  ROS_DEBUG("Drum top %.3f", upper_bin_range);
  return upper_bin_range;
}

void publishDrumCollisionObject( Eigen::VectorXf &drum_params){
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::CollisionObject drum_collision_obj;
  geometry_msgs::Pose drum_pose;
  
  // Check if mesh model is loaded into file
  // std::map<std::string, moveit_msgs::CollisionObject> collision_objects = planning_scene_interface.getObjects( std::vector<std::string>{"drum.stl"} );
  // ROS_DEBUG_STREAM("Objects found " << collision_objects.size());
  
  drum_collision_obj.header.frame_id = "/base_link";
  drum_collision_obj.header.stamp = ros::Time::now();
  // The id of the object is used to identify it.
  drum_collision_obj.id = "drum.stl";
  drum_collision_obj.operation = drum_collision_obj.ADD;
  
  const Eigen::Vector3d scale_vector(drum_params[6],drum_params[6],drum_params[7]);
  shapes::Mesh* m = shapes::createMeshFromResource("package://invite_vision/meshes/drum.stl", scale_vector); 
  // Define and load the mesh
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;  
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  ROS_INFO_ONCE("Drum mesh...loaded");

  drum_collision_obj.meshes.resize(1);
  drum_collision_obj.mesh_poses.resize(1);

  tf::Quaternion orientation;
  orientation.setRPY(0.0 , 0.0 , 0.0);        // Asume Drum axis is the robot Z axis 
  drum_pose.orientation.x = orientation.x();
  drum_pose.orientation.y = orientation.y();
  drum_pose.orientation.z = orientation.z();
  drum_pose.orientation.w = orientation.w();
  drum_pose.position.x = drum_params[0];
  drum_pose.position.y = drum_params[1];
  drum_pose.position.z = 0;   
  
  drum_collision_obj.meshes.push_back(mesh);
  drum_collision_obj.mesh_poses.push_back(drum_pose);
  
  drum_collision_obj.mesh_poses[0] = drum_pose;

  // std::vector<moveit_msgs::CollisionObject> updated_collision_objects;
  // updated_collision_objects.push_back(drum_collision_obj);

  ROS_INFO("Applying collision cylinder shape");
  planning_scene_interface.applyCollisionObject(drum_collision_obj);

  // After object is placed into planning scene, publish drum parameters on topic
  invite_vision::DrumParams params_msg;
  params_msg.header.stamp = ros::Time::now();
  params_msg.x = drum_params[0];
  params_msg.y = drum_params[1];
  params_msg.height = drum_params[7];
  params_msg.radius = drum_params[6];
  params_msg.stable = drum_found;
  drum_params_pub.publish( params_msg );
  if( drum_found )
    drum_params_pub.shutdown();
}

void parameter_update(invite_vision::DrumFinderConfig &config, uint32_t level) {
  ROS_INFO("Updating parameters \nMax iterations: %d\nEps Angle: %f\nNormals Weight: %f\nDistance Theshold: %f\nz_normal_weight: %f", config.max_iterations, config.eps_angle, config.normals_weight, config.distance_threshold, config.z_normal_weight);
  max_iterations = config.max_iterations;
  eps_angle = config.eps_angle;
  normals_weight = config.normals_weight;
  distance_threshold = config.distance_threshold;
  z_normal_weight = config.z_normal_weight;
}
