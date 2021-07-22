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
CW3::makeBox(std::string id, std::string frame_id, Box box)
{
  // Makes a Box collision object at given location with given dimensions. 

  moveit_msgs::CollisionObject collision_object;
  
  // input header information
  collision_object.id = id;
  collision_object.header.frame_id = frame_id;

  /* Define the primitive and its dimensions. */
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = box.length;
  collision_object.primitives[0].dimensions[1] = box.width;
  collision_object.primitives[0].dimensions[2] = box.height;

  // determine orientation
  tf::Quaternion q;
  geometry_msgs::Quaternion q_msg;
  q.setRPY(box.roll, box.pitch, box.yaw);
  tf::quaternionTFToMsg (q, q_msg);
  
  /* Define the pose of the table: center of the cube. */
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x =  box.x;
  collision_object.primitive_poses[0].position.y =  box.y;
  collision_object.primitive_poses[0].position.z =  box.z;
  collision_object.primitive_poses[0].orientation = q_msg;

  collision_object.operation = collision_object.ADD;
  return collision_object;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::makeTable(std::string id, std::string frame_id, Table table,
  std::vector<moveit_msgs::CollisionObject>& collision_objects)
{
  // Makes a Box collision object at given location with given dimensions. 

  // inspect the incoming vector (passed by reference)
  int i = collision_objects.size();

  // extend collision object vector by number of boxes required
  if (table.shelf) 
  {
    collision_objects.resize(i + 4);
  }
  else 
  {
    collision_objects.resize(i + 1);
  }

  // create the base of the table
  collision_objects[i] = makeBox (id, frame_id, table.base);

  // if we have a shelf, create this
  if (table.shelf)
  {

    /* First, we need to rotate the shelf elements to correct orientation */
    Eigen::Matrix3d rotation_matrix;

    // generate the correct rotation matrix for this table
    rotation_matrix = Eigen::AngleAxisd (table.roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd (table.pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd (table.yaw, Eigen::Vector3d::UnitZ());

    // create unit vectors and rotate them
    Eigen::Vector3d left_eigen(table.left.x, table.left.y, table.left.z);
    Eigen::Vector3d right_eigen(table.right.x, table.right.y, table.right.z);
    left_eigen = rotation_matrix * left_eigen;
    right_eigen = rotation_matrix * right_eigen;

    // save the rotated co-ordinates, then add in the table base position
    table.left.x = left_eigen(0) + table.base.x;
    table.left.y = left_eigen(1) + table.base.y;
    table.left.z = left_eigen(2) + table.base.z;

    table.right.x = right_eigen(0) + table.base.x;
    table.right.y = right_eigen(1) + table.base.y;
    table.right.z = right_eigen(2) + table.base.z;

    table.top.x += table.base.x;
    table.top.y += table.base.y;
    table.top.z += table.base.z;

    // make the shelf collision objects, adding them to the vector
    collision_objects[i+1] = makeBox (id + "_left", frame_id, table.left);
    collision_objects[i+2] = makeBox (id + "_right", frame_id, table.right);
    collision_objects[i+3] = makeBox (id + "_top", frame_id, table.top);
  }
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& 
                       planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  makeTable ("table1", "panda_link0", table_1_, collision_objects);
  makeTable ("table2", "panda_link0", table_2_, collision_objects);
  makeTable ("table3", "panda_link0", table_3_, collision_objects);

  // create the object
  Box object_box {table_2_.x, table_2_.y, table_2_.table_height + 0.1,
                  0, 0, 0,
                  0.02, 0.02, 0.2};

  collision_objects.push_back (makeBox ("object", "panda_link0", object_box));

  planning_scene_interface.applyCollisionObjects (collision_objects);
}
