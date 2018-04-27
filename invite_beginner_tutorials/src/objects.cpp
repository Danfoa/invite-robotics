/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman 
   Modified by: Daniel Ordonez Apraez - daniels.ordonez@gmail.com
/* 
This document is a modification of the Move Grup Moveit Tutorial, in order 
for it to work with CSDA10F robot, it is intended to familiarize the operator with several 
Moveit classes that are commonly used, and most of the "common robot 
operations".

We formaly thank SwRI for ofering this code under the BSD licence and as 
requested by the software license agreement we state that we do not have 
any relationship with SwRI and that they do not hold any responsability whatsoever over
this code.

*/ 

//For add object
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
using namespace Eigen;
//********

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Define a collision object ROS message.

   moveit::planning_interface::PlanningSceneInterface current_scene;
   Vector3d b(0.001, 0.001, 0.001); // vector para escalar
   moveit_msgs::CollisionObject co;


  // The id of the object is used to identify it.
   co.id = "glass";

    
    shapes::Mesh* m = shapes::createMeshFromResource("package://invite_beginner_tutorials/meshes/glass.stl",b); 
    ROS_INFO("Glass mesh loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;  
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    co.meshes.resize(1);
    co.mesh_poses.resize(1);
    co.meshes[0] = mesh;
    co.header.frame_id = "glass";

   
    co.mesh_poses[0].position.x = 0.75;
    co.mesh_poses[0].position.y = -0.7;
    co.mesh_poses[0].position.z = 0.9;
    co.mesh_poses[0].orientation.w= 1.0; 
    co.mesh_poses[0].orientation.x= 0.0; 
    co.mesh_poses[0].orientation.y= 0.0;
    co.mesh_poses[0].orientation.z= 0.0;

    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    std::vector<moveit_msgs::CollisionObject> vec;
    vec.push_back(co);
    ROS_INFO("glass added into the world");
    current_scene.addCollisionObjects(vec);
//***************************************************


  ros::shutdown();
  return 0;
}
