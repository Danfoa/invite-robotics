/*
 * This file is part of invite_utils package.
 *
 * To-Do: Add licence description
 * 
 * File name: vision_system_interface.h
 * 
 * Description: 
 *        This header defines the necessary classes, constants and functions to easily operate, and process the
 *        data incomming from the two IDS N35 cameras.
 *          
 *        The `VisionInterface` class should allow the user to:
 *             - Request single or dual camera data (continuously or on user request)
 *             - Query the current state of the system (busy, idle, error, etc)
 * 
 * Author: 
 *        Daniel Felipe Ordonez Apraez - daniels.ordonez@gmail.com - OrdonezApraez@invite-research.com  
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <ensenso_camera_msgs/RequestDataAction.h>
#include <ensenso_camera_msgs/RequestDataFeedback.h>
#include <ensenso_camera_msgs/RequestDataGoal.h>
#include <ensenso_camera_msgs/RequestDataResult.h>

typedef ensenso_camera_msgs::RequestDataAction RequestDataAction;
typedef ensenso_camera_msgs::RequestDataGoal RequestDataGoal; 
typedef ensenso_camera_msgs::RequestDataResult RequestDataResult; 

namespace invite_vision{
    /** Enum type representing the possible operation stages of the camera system
     */
    enum class SystemStatus {
        ERROR = 0,
        IDLE = 1,
        BUSY = -1,
        INITALIZING = -2        
    };

    /** Provides a high level control over the cameras system, managing the connection to the
     * cameras action servers, requesting pointclouds and monitoring the operation status. 
     * Example:  
     *   invite_vision::VisionInterface vision_system_interface;
     *
     *  if (vision_system_interface.system_status != invite_vision::SystemStatus::IDLE)
     *      ROS_ERROR("Camera system initialization failed")
     *   else 
     *       vision_system_interface.requestDualCameraData();
     */
    class VisionInterface{

        private:
            // Cameras action clients
            actionlib::SimpleActionClient<RequestDataAction> * upper_camera_client_;
            actionlib::SimpleActionClient<RequestDataAction> * lower_camera_client_;
            // Type of camera data request
            RequestDataGoal default_data_goal_;
        public:
            // Status of both cameras as a hole system
            SystemStatus system_status = SystemStatus::INITALIZING;
            // Independent camera status
            SystemStatus upper_camera_status = SystemStatus::INITALIZING;
            SystemStatus lower_camera_status = SystemStatus::INITALIZING;
            
            VisionInterface(){
                // Initialize camera action clients
                upper_camera_client_ = new actionlib::SimpleActionClient<RequestDataAction>("/n35_upper_camera/request_data", true);
                lower_camera_client_ = new actionlib::SimpleActionClient<RequestDataAction>("/n35_lower_camera/request_data", true);
                
                // To-Do: Make user defined !.
                default_data_goal_.publish_results = true;
                default_data_goal_.request_point_cloud = true;
                default_data_goal_.request_normals = true;
                
                // Wait for camera action server to be ready
                ROS_INFO("Waiting for camera action servers");
                if(upper_camera_client_->waitForServer(ros::Duration(10)) && lower_camera_client_->waitForServer(ros::Duration(10))){
                    ROS_DEBUG("Cameras action servers are ready for data request");  
                    system_status = SystemStatus::IDLE;
                    lower_camera_status = SystemStatus::IDLE;
                    upper_camera_status = SystemStatus::IDLE;
                }else{
                    system_status = lower_camera_status = upper_camera_status = SystemStatus::ERROR;
                    const char *error_msg = "Camera server not running\nPlease check that \n1. Both cameras are powered on.\n"
                        "2. Both upper and lower camera_nodes are running and that connection to the camera was achieved\n" 
                        "3. Both cameras have their firmware updated (open nxView to confirm this or update firmware)";
                    ROS_FATAL("%s", error_msg);
                    throw(error_msg);
                }  
            }

            // Request single camera data from both cameras.
            void requestDualCameraData(){
                if( system_status == SystemStatus::IDLE){
                    ROS_DEBUG("Requesting dual camera data");
                    upper_camera_client_->sendGoal( default_data_goal_, boost::bind(&VisionInterface::upperCameraDataReceivedCallback, this, _1, _2));
                    lower_camera_client_->sendGoal( default_data_goal_, boost::bind(&VisionInterface::lowerCameraDataReceivedCallback, this, _1, _2));
                    upper_camera_status = SystemStatus::BUSY;
                    lower_camera_status = SystemStatus::BUSY;
                    system_status = SystemStatus::BUSY;
                } else {
                    ROS_ERROR("Request rejected: Previous data request has not been processed yet");
                }
            }
            
            // Upper camera action client result callback 
            void upperCameraDataReceivedCallback(const actionlib::SimpleClientGoalState& state, const RequestDataResult::ConstPtr& result){
                upper_camera_status = SystemStatus::IDLE;   // Set camera as ready to receive new goals now.
                if( state == actionlib::SimpleClientGoalState::SUCCEEDED ){
                    ROS_DEBUG("Upper camera request succeded");
                    // TO-DO: Setup user defined callback function for processing data directly 
                    updateSystemStatus();
                }else{
                    ROS_ERROR("Upper camera request failed, goals status: %s",  state.toString().c_str());
                }
            }

            // Lower camera action client result callback 
            void lowerCameraDataReceivedCallback(const actionlib::SimpleClientGoalState& state, const RequestDataResult::ConstPtr& result){
                lower_camera_status = SystemStatus::IDLE;   // Set camera as ready to receive new goals now.
                if( state == actionlib::SimpleClientGoalState::SUCCEEDED ){
                    ROS_DEBUG("Lower camera request succeded");
                    // TO-DO: Setup user defined callback function for processing data directly 
                    updateSystemStatus();
                }else{
                    ROS_ERROR("Lower camera request failed, goals status: %s",  state.toString().c_str());
                }
            }


        private:
            // Modify state variable for both cameras a single system.
            void updateSystemStatus(){
                if (lower_camera_status == SystemStatus::IDLE && upper_camera_status == SystemStatus::IDLE){
                    system_status = SystemStatus::IDLE;
                    ROS_DEBUG("Vision system ready for new request");
                }
                if (lower_camera_status == SystemStatus::BUSY || upper_camera_status == SystemStatus::BUSY)
                    system_status = SystemStatus::BUSY;
            }

    };
}