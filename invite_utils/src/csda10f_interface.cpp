
#include "invite_utils/csda10f_interface.h"

namespace invite_utils{

  CSDA10F::CSDA10F(float max_speed = 0.1) : csda10f_mg("csda10f"),
                                    arms_mg("arms"),
                                    torso_mg("torso"),
                                    left_arm_mg("arm_left"),
                                    left_arm_with_torso_mg("arm_left_with_torso"),
                                    right_arm_mg("arm_right"),
                                    right_arm_with_torso_mg("arm_right_with_torso"){
    nh_.reset(new ros::NodeHandle());

    // Robot configuration
    max_velocity_scaling_factor = max_speed;
    // Start gripper action servers.
    left_gripper = new RobotiqActionClient("/left_gripper/command_robotiq_action", true);
    right_gripper = new RobotiqActionClient("/right_gripper/command_robotiq_action", true);
    // Service client for FT300 sensor
    left_ft300_srv_ = nh_->serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc", true);

    // Check current robot state is obtainable.
    if ( !csda10f_mg.startStateMonitor() ) {
      ROS_FATAL("Robot state monitor was not created, restart robot firmware");
      throw("Robot state monitor was not created, restart robot firmware");
    }

    current_robot_state_ptr = csda10f_mg.getCurrentState(3.0);
    configureRobotInterface();
    // Connect to YASKAWAS robot state messages
    ros::Subscriber sub = nh_->subscribe<industrial_msgs::RobotStatus>("/robot_status",
                                                                      1,
                                                                      &CSDA10F::updateRobotStatus,
                                                                      this);
    //  Generate client to enable/dissable robot servo-power / motions
    robot_enabler_srv_ = nh_->serviceClient<std_srvs::Trigger>("/robot_enable", true);
    robot_disabler_srv_ = nh_->serviceClient<std_srvs::Trigger>("/robot_enable", true);
    stop_motion_srv_ = nh_->serviceClient<industrial_msgs::StopMotion>("/stop_motion", true);
    enable_robot();

    ROS_WARN("Robot ready for motion at %.1f%% of its max speed", max_velocity_scaling_factor * 100);

    // Publishers, subscribers, visual tools and services config
    sound_pub = nh_->advertise<sound_play::SoundRequest>("/robotsound", 3);
    talkative = true;
    psm_.reset( new planning_scene_monitor::PlanningSceneMonitor("robot_description") );
    // psm_->requestPlanningSceneState();
    psm_->publishDebugInformation(true);
    psm_->startSceneMonitor("/move_group/monitored_planning_scene");

    planning_scene_diff_client_ = nh_->serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client_.waitForExistence();
    // Visualization functions
    visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/rviz_visual_tools", psm_));
    // visual_tools->loadPlanningSceneMonitor();
    visual_tools->loadMarkerPub(true);
    visual_tools->loadRobotStatePub("/display_robot_state");
    visual_tools->loadTrajectoryPub("/move_group/display_planned_path");
    visual_tools->loadSharedRobotState();
    visual_tools->enableBatchPublishing();
    visual_tools->deleteAllMarkers();
    // visual_tools->hideRobot();
    visual_tools->trigger();
    // visual_tools->setManualSceneUpdating();

    // Load default collision scene if path is given
    std::string collision_scene_path, param_key;
    if (nh_->searchParam("csda10f/collision_scene", param_key)) {
      nh_->getParam(param_key, collision_scene_path);
      ROS_WARN("Loading collision scene (%s)", collision_scene_path.c_str());
      visual_tools->loadCollisionSceneFromFile(collision_scene_path);
    }else
      ROS_WARN("No collision scene loaded");

    visual_tools->trigger();
    // Planning Scene configuration
    robot_model_ = psm_->getRobotModel();
  }

  void CSDA10F::updateRobotStatus( const industrial_msgs::RobotStatusConstPtr& new_status) {
      if (new_status->in_error.val == industrial_msgs::TriState::ON )
        ROS_ERROR("Robot error: alarm % is active", new_status->error_code);
      if (new_status->in_motion.val == false && status.in_motion.val == true){
        current_robot_state_ptr = csda10f_mg.getCurrentState();
        ROS_DEBUG_STREAM("-- Robot State: Robot has just stopped its motion " << current_robot_state_ptr );
      }
      if (new_status->motion_possible.val == false && status.motion_possible.val == true)
        ROS_ERROR("Robot motion was disabled");
      this->status = *new_status;
  }

  void CSDA10F::configureRobotInterface() {
    // Set Velocity Scaling Factor
    csda10f_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    arms_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    left_arm_with_torso_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    right_arm_with_torso_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    left_arm_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    right_arm_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    torso_mg.setMaxVelocityScalingFactor(max_velocity_scaling_factor * 2);

    // Set Joint orientation tolerance (This is needed to cope with the `joint_trajectory_streamer` start
    // position tolerance limit `start_pos_tol_`
    // csda10f_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
    // arms_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
    // left_arm_with_torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
    // right_arm_with_torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
    // left_arm_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
    // right_arm_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
    // torso_mg.setGoalJointTolerance( joint_goal_orientation_tolerance );
    speak("I am ready");
  }

  // Enable robot servomotor motions
  bool CSDA10F::enable_robot() {
    if (robot_enabler_srv_) {
      ROS_WARN("Enabling Robot");
      std_srvs::Trigger cmd;
      robot_enabler_srv_.call(cmd);
      if(cmd.response.success)
        ROS_WARN("Robot is now enabled for motion");
      else
        ROS_ERROR("Robot was not enabled: %s", cmd.response.message.c_str());
    }
  }

  // Disable robot servomotor motions
  bool CSDA10F::disable_robot() {
    if (robot_disabler_srv_) {
      ROS_WARN("Disabling Robot");
      std_srvs::Trigger cmd;
      robot_enabler_srv_.call(cmd);
      if(!cmd.response.success)
        ROS_ERROR("Robot was not disabled: %s", cmd.response.message.c_str());
    }
  }

  // Stop robot motion
  bool CSDA10F::stopMotion() {
    ROS_WARN("Stopping robot motion");
    if (stop_motion_srv_) {
      industrial_msgs::StopMotion cmd;
      // robot_enabler_srv_.call(cmd);
      if (cmd.response.code.val == 1)
        ROS_INFO("CSDA10F Interface: Robot motion stopped");
      else
        ROS_ERROR("CSDA10F Interface: Error while stopping robot motion");
    }
    // Notify Moveit of the cancel/stop request
    csda10f_mg.stop();
    arms_mg.stop();
    left_arm_with_torso_mg.stop();
    right_arm_with_torso_mg.stop();
    left_arm_mg.stop();
    right_arm_mg.stop();
    torso_mg.stop();
  }

  bool CSDA10F::goHome(std::string home_pose_name = "home") {
    if (!status.motion_possible.val == true)
      enable_robot();
    if (talkative)
      speak("Going to " + home_pose_name + " position");
    // Take left arm home'
    csda10f_mg.setNamedTarget(home_pose_name);
    error_code = csda10f_mg.move();
    if (error_code != MoveItErrorCode::SUCCESS){
      ROS_ERROR("Motion to position (%s) impossible, error code: (%d) %s",
                  home_pose_name.c_str(),
                  (int)error_code.val,
                  getErrorMsg(error_code).c_str()
                  );
      return false;
    }
    ros::Duration(0.2).sleep();
    return true;
  }

  bool CSDA10F::tareLeftForceSensor() {
    robotiq_ft_sensor::sensor_accessor command;
    command.request.command_id = robotiq_ft_sensor::sensor_accessor::Request::COMMAND_SET_ZERO;
    if(left_ft300_srv_.exists()) {
      left_ft300_srv_.call(command.request, command.response);
      return command.response.success;
    } else {
      ROS_ERROR("Left force sensor tare service [%s] is not available",
                left_ft300_srv_.getService().c_str());
      return false;
    }
  }

  void CSDA10F::speak(std::string msg) {
    if (!talkative) {
      ROS_INFO("CSDA10F: %s", msg.c_str());
      return;
    }
    sound_play::SoundRequest req;
    req.sound = req.SAY;
    req.command = req.PLAY_ONCE;
    req.volume = 1.0;
    req.arg = msg;
    // ROS_INFO("Speaking : %s" , msg.c_str());
    sound_pub.publish(req);
  }

  // Static functions
  // -------------------------------------------------------------------------------------
  
  trajectory_msgs::JointTrajectory CSDA10F::getRightGripperPosture(float distance_between_fingers) {
    trajectory_msgs::JointTrajectory posture;
    posture.header.stamp = ros::Time::now();
    posture.joint_names = std::vector<std::string>{"right_gripper_finger_joint"};
    trajectory_msgs::JointTrajectoryPoint close_state;

    float joint_position = 0.8 - (0.8/0.085 * distance_between_fingers);
    joint_position = joint_position > 0.8 ? 0.8 : joint_position;
    joint_position = joint_position < 0.0 ? 0.0 : joint_position;

    close_state.positions.push_back(joint_position);
    close_state.velocities.push_back(0.05f);
    close_state.effort.push_back(10.0);
    close_state.time_from_start = ros::Duration(4.5);
    posture.points.push_back(close_state);
    return posture;
  }

  trajectory_msgs::JointTrajectory CSDA10F::getLeftGripperPosture(float distance_between_fingers) {
    trajectory_msgs::JointTrajectory posture;
    posture.header.stamp = ros::Time::now();
    posture.joint_names = std::vector<std::string>{"left_gripper_finger_joint"};
    trajectory_msgs::JointTrajectoryPoint close_state;

    float joint_position = 0.8 - (0.8/0.085 * distance_between_fingers);
    joint_position = joint_position > 0.8 ? 0.8 : joint_position;
    joint_position = joint_position < 0.0 ? 0.0 : joint_position;

    close_state.positions.push_back(joint_position);
    close_state.velocities.push_back(0.05f);
    close_state.effort.push_back(10.0);
    close_state.time_from_start = ros::Duration(4.5);
    posture.points.push_back(close_state);
    return posture;
  }

  std::string CSDA10F::getErrorMsg(MoveItErrorCode error_code){
    std::string msg;
    switch(error_code.val){
      case 1: msg = "Success"; break;
      case 99999: msg = "Failure"; break;
      case -1: msg = "Planning failed"; break;
      case -2: msg = "Invalid motion plan"; break;
      case -3: msg = "Motion plan invalidated by environment change"; break;
      case -4: msg = "Control failed"; break;
      case -5: msg = "Unable to acquire sensor data"; break;
      case -6: msg = "Time out reach"; break;
      case -7: msg = "Preempted"; break;
      case -10: msg = "Start state is in collision"; break;
      case -11: msg = "Start state violates path constraints"; break;
      case -12: msg = "Goal state is in collision"; break;
      case -13: msg = "Goal state violates path constraints"; break;
      case -14: msg = "Goal constraints violated"; break;
      case -15: msg = "Invalid group name"; break;
      case -16: msg = "Invalid goal constraints"; break;
      case -17: msg = "Invalid robot state"; break;
      case -18: msg = "Invalid link name"; break;
      case -19: msg = "Invalid object name"; break;
      case -21: msg = "Frame transform failure"; break;
      case -22: msg = "Collision checking unavailable"; break;
      case -23: msg = "Robot state stale"; break;
      case -24: msg = "Sensor info stale"; break;
      case -31: msg = "No Inverse Kinematics solution found"; break;
      default: msg = "Unknown error code";
    };
    return msg;
  }

}

