/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-, Dimitrios Kanoulas
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include <cw3.h>

CW3::CW3 (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;
  initParams ();
  updateParams (nh);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::initParams ()
{
  // Frames
  this->world_frame_ = "/world_frame";
  
  // Topics
  this->robot_frame_ = "/robot_frame";
  
  // HW1-Q2: setup the names for frames {0}, {1}, {2}, and {3}
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::updateParams (ros::NodeHandle &nh)
{
  // Frames
  nh.getParam("/frame_params/world_frame", this->world_frame_);
  nh.getParam("/frame_params/robot_frame", this->robot_frame_);
}


////////////////////////////////////////////////////////////////////////////////
std::string
CW3::getWorldFrame ()
{
  return (this->world_frame_);
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW3::getRobotFrame ()
{
  return (this->robot_frame_);
}

////////////////////////////////////////////////////////////////////////////////
int
CW3::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::Lab1CreateFrames ()
{
  // generate a robot frame attached to the world frame (0,0,0)
  transf_.setOrigin (tf::Vector3(0.0, 0.0, 0.0));
  transf_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //quaternion
  
  // Note that the rotation can be given in various forms (rotation matrix,
  // axis-angle, etc), but the function setRotation gets a tf::Quaternion,
  // thus when a different type rotation is available, it should be converted
  // to a tf::Quaternion.
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::Lab1PublishFrames ()
{
  // publish world->robot
  tranf_br_.sendTransform(tf::StampedTransform(transf_,
                                               ros::Time::now(), 
                                               world_frame_,
                                               robot_frame_));
}

////////////////////////////////////////////////////////////////////////////////
moveit_msgs::CollisionObject
CW3::cw1Q3MakeBox(std::string id, std::string frame_id,
					float dim_x, float dim_y, float dim_z,
					float pos_x, float pos_y, float pos_z)
{
  // Makes a Box collision object at given location with given dimensions. 

  moveit_msgs::CollisionObject collision_object;
  
  // Add the first table where the cube will originally be kept.
  collision_object.id = id;
  collision_object.header.frame_id = frame_id;
  
  /* Define the primitive and its dimensions. */
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dim_x;
  collision_object.primitives[0].dimensions[1] = dim_y;
  collision_object.primitives[0].dimensions[2] = dim_z;

  /* Define the pose of the table: center of the cube. */
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x =  pos_x;
  collision_object.primitive_poses[0].position.y =  pos_y;
  collision_object.primitive_poses[0].position.z =  pos_z;

  collision_object.operation = collision_object.ADD;
  return collision_object;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& 
                       planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 4 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);
  
  // Add the first table where the cube will originally be kept.
  collision_objects[0] = CW3::cw1Q3MakeBox("table1", "panda_link0",
                                            0.4, 0.2, 0.4,
                                            0.0, 0.5, 0.2);
  
  // Add the second table where we will be placing the cube.
  collision_objects[1] = CW3::cw1Q3MakeBox("table2", "panda_link0",
                                            0.2, 0.4, 0.4,
					    -0.5, 0.0, 0.2);
  
  // Add the second table where we will be placing the cube.
  collision_objects[2] = CW3::cw1Q3MakeBox("table3", "panda_link0",
                                            0.4, 0.2, 0.4,
                                            0.0, -0.5, 0.2);

  // Define the object that we will be manipulating
  collision_objects[3] = CW3::cw1Q3MakeBox("object", "panda_link0",
                                            0.02, 0.02, 0.2,
                                            -0.5, 0.0, 0.5);

  planning_scene_interface.applyCollisionObjects(collision_objects);
}
