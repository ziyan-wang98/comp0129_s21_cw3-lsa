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

#ifndef CW3_H_
#define CW3_H_

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// HW1 Includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

// cw3 Includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>


/** \brief CW 3.
  *
  * \author Dimitrios Kanoulas
  */
class CW3
{
  public:
 
    /** \brief Structure to hold collision box information */
    struct Box {

      double x, y, z;
      double roll, pitch, yaw;
      double length;
      double width;
      double height;

      // constructor
      Box(double x, double y, double z, double roll, double pitch, double yaw,
          double length, double width, double height)
          : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw),
            length(length), width(width), height(height) {}
    };

    /** \brief Structure to hold table information */
    struct Table {

      // table position parameters
      double x,y,z;
      double roll,pitch,yaw;

      // constants that determine table sizes
      static constexpr double table_length = 0.4;
      static constexpr double table_height = 0.4;
      static constexpr double table_width = 0.2;
      static constexpr double wall_thickness = 0.05;
      static constexpr double shelf_height = 0.3;

      // does this table have a shelf on top
      bool shelf;

      // create boxes for the table and shelf
      Box base{x, y, z + table_height/2,
        roll, pitch, yaw,
        table_length, table_width, table_height};

      Box left{-table_length/2 + wall_thickness/2, 0, table_height/2 + shelf_height/2,
        roll, pitch, yaw,
        wall_thickness, table_width, shelf_height};

      Box right{table_length/2 - wall_thickness/2, 0, table_height/2 + shelf_height/2,
        roll, pitch, yaw,
        wall_thickness, table_width, shelf_height};

      Box top{0, 0, table_height/2 + shelf_height - wall_thickness/2,
        roll, pitch, yaw,
        table_length, table_width, wall_thickness};

      // constructor
      Table(double x, double y, double z, double roll, double pitch, double yaw, bool shelf)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw), shelf(shelf) {}
    };
 
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    CW3 (ros::NodeHandle &nh);
    
    /** \brief Lab 1: initialize the parameters. */
    void
    initParams ();
    
    /** \brief Load the parameters. */
    void
    updateParams (ros::NodeHandle &nh);
    
    /** \brief Lab 1: get the world frame.
      *
      * \return the world frame string
      */
    std::string
    getWorldFrame ();
    
    /** \brief Lab 1: get the robot frame.
      *
      * \return the robot frame string
      */
    std::string
    getRobotFrame ();
    
    /** \brief Lab 1: create the transformation between robot & world
     *  frames. 
     */
    void
    Lab1CreateFrames ();
    
    /** \brief Lab 1: publish the transformations between robot & world
      * frames.
      */
    void
    Lab1PublishFrames ();
    
    /** \brief CW1: helper function to implement a non-blocking key getter.
      *
      * \return 1 for key pressed, 0 for no pressed
      */
    int
    kbhit ();
    
    /** \brief helper function to create a collision object
      *
      * \input id name of the collision object
      * \input frame_id base frame of the collision object
      * \input box structure containing details of the collision object
      */
    moveit_msgs::CollisionObject
    makeBox(std::string id, std::string frame_id, Box box);
    
    /** \brief helper function to create a table in the scene
      * \input id name of the collision object
      * \input frame_id base frame of the collision object
      * \input table structure containing details of the table object
      * \input collision_objects vector of collision objects which is added to
      */
    void
    makeTable(std::string id, std::string frame_id, Table table,
      std::vector<moveit_msgs::CollisionObject>& collision_objects);

    /** \brief CW1 Q2: add collision objects: table1, table2, table3, object
      *
      * \input planning_scene_interface the MoveIt! PlanningSceneInterface
      */
    void
    cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

  public:
    /** \brief Node handle. */
    ros::NodeHandle nh_;
    
    /** \brief World and robot frames. */
    std::string world_frame_, robot_frame_;
    
    /** \brief Lab 1: TF transforms definition. */
    tf::Transform transf_;
    
    /** \brief Lab 1: TF transform broadcaster definitions. */
    tf::TransformBroadcaster tranf_br_;
 
    /** \brief create the tables at specific positions and orientations. */
    Table table_1_{ 0.1,  0.6,  0, 0, 0, 0,      false};
    Table table_2_{-0.5,  0.25, 0, 0, 0, 1.5708, true};
    Table table_3_{-0.4, -0.4,  0, 0, 0, 2.3562, true}; 
    
  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif
