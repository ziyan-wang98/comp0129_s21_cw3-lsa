/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Ridhwan Luthra.
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
*   * Neither the name of Ridhwan Luthra nor the names of its
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

/* Author: Ridhwan Luthra */
/* Modified author: Dimitrios Kanoulas */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

// Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>


#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

class CylinderSegment
{
public:
  CylinderSegment ()
  {
    // ROS node generation
    ros::NodeHandle nh;
    
    // Initialize subscriber to the raw point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1,
                                        &CylinderSegment::cloudCB, this);
    
    
    // Subscriber:
    //subscriber 'v' button to switch VoxelGrid filter.
    ros::Subscriber sub_vg_key = nh.subscribe("/vg_filter", 1,
                            &CylinderSegment::switch_vg_flag, this);
    //subscriber 'c' button to switch CropBox filter.
    ros::Subscriber sub_cb_key = nh.subscribe("/cb_filter", 1,
                            &CylinderSegment::switch_cb_flag, this);
    //subscriber 's' button to switch statistic outliers filter.
    ros::Subscriber sub_sor_key = nh.subscribe("/sor_filter", 1,
                            &CylinderSegment::switch_sor_flag, this);
                        
    //Chech grasp states
    ros::Subscriber sub_grasp_states = nh.subscribe("/grasp_status", 1, 
                        &CylinderSegment::set_grasp_status, this); 

    // Publisher:
    //Pose Publisher
    pub_cylinder_pose = nh.advertise<geometry_msgs::PoseStamped>("/cylinder_pose", 1, true);
    //Radius Publisher
    pub_cylinder_radius = nh.advertise<std_msgs::Float64>("/cylinder_radius", 1, true);
    //Height Publisher
    pub_cylinder_height = nh.advertise<std_msgs::Float64>("/cylinder_height", 1, true);

    // Spin
    ros::spin ();
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the parameters of the cylinder add the cylinder to the
    * planning scene.
    *
    * @param cylinder_params - Pointer to the struct AddCylinderParams.
    */
  void addCylinder ()
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // BEGIN_SUB_TUTORIAL add_cylinder
    //
    // Adding Cylinder to Planning Scene
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "camera_rgb_optical_frame";
    collision_object.id = "cylinder";

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    /* Setting height of cylinder. */
    primitive.dimensions[0] = cylinder_params->height;
    /* Setting radius of cylinder. */
    primitive.dimensions[1] = cylinder_params->radius;

    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose cylinder_pose;
    /* Computing and setting quaternion from axis angle representation. */
    Eigen::Vector3d cylinder_z_direction(cylinder_params->direction_vec[0],
                                         cylinder_params->direction_vec[1],
                                         cylinder_params->direction_vec[2]);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
    cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
    cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
    cylinder_pose.orientation.w = cos(angle / 2);

    // Setting the position of cylinder.
    cylinder_pose.position.x = cylinder_params->center_pt[0];
    cylinder_pose.position.y = cylinder_params->center_pt[1];
    cylinder_pose.position.z = cylinder_params->center_pt[2];

    // Add cylinder as collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);
    // END_SUB_TUTORIAL
    
    // Publish Pose
    publishPose(cylinder_pose);
    // Publish Radius and Height
    std_msgs::Float64 radius_msg, height_msg;
    radius_msg.data = cylinder_params->radius;
    height_msg.data = cylinder_params->height;
    pub_cylinder_radius.publish(radius_msg);
    pub_cylinder_height.publish(height_msg);
  }
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pose of the cylinder and publish it by publisher.
    *
    * @param cylinder_pose - Pointer to the Pose(position and orientation).
    */
  
  void publishPose (geometry_msgs::Pose cylinder_pose)
  {
    
    // Position
    tf::Vector3 p;
    p.setX(cylinder_pose.position.x);
    p.setY(cylinder_pose.position.y);
    p.setZ(cylinder_pose.position.z);
    // Orientation
    tf::Quaternion q;
    q.setX(cylinder_pose.orientation.x);
    q.setY(cylinder_pose.orientation.y);
    q.setZ(cylinder_pose.orientation.z);
    q.setW(cylinder_pose.orientation.w);

    // cylinder transform to camera frame
    tf::StampedTransform Trans_C2C;
    Trans_C2C.setRotation(q);
    Trans_C2C.setOrigin(p);
    transformListener.lookupTransform("/panda_link0", "/camera_rgb_optical_frame",
                                                     ros::Time(0), Trans_C2B);
    Trans_C2B.setData(Trans_C2B * Trans_C2C);

    // Set pose message
    geometry_msgs::PoseStamped cylinder_msg;
    cylinder_msg.pose.position.x = Trans_C2B.getOrigin().getX();
    cylinder_msg.pose.position.y = Trans_C2B.getOrigin().getY();
    cylinder_msg.pose.position.z = Trans_C2B.getOrigin().getZ();
    cylinder_msg.pose.orientation.x = Trans_C2B.getRotation().getX();
    cylinder_msg.pose.orientation.y = Trans_C2B.getRotation().getY();
    cylinder_msg.pose.orientation.z = Trans_C2B.getRotation().getZ();
    cylinder_msg.pose.orientation.w = Trans_C2B.getRotation().getW();

    pub_cylinder_pose.publish(cylinder_msg);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud containing just the cylinder, compute its
    *        center point and its height and store in cylinder_params.
    *
    *  @param cloud - Pointcloud containing just the cylinder.
    *  @param cylinder_params - Pointer to the struct AddCylinderParams.
    */
  void extractLocationHeight (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    double max_angle_y = 0.0;
    double min_angle_y = std::numeric_limits<double>::infinity();

    double lowest_point[3];
    double highest_point[3];
    // BEGIN_SUB_TUTORIAL extract_location_height
    // Consider a point inside the point cloud and imagine that point is formed
    // on a XY plane where the perpendicular distance from the plane to the
    // camera is Z. |br|
    // The perpendicular drawn from the camera to the plane hits at center of
    // the XY plane. |br|
    // We have the x and y coordinate of the point which is formed on the XY
    // plane. |br|
    // X is the horizontal axis and Y is the vertical axis. |br|
    // C is the center of the plane which is Z meter away from the center of
    // camera and A is any point on the plane. |br|
    // Now we know Z is the perpendicular distance from the point to the
    // camera. |br|
    // If you need to find the  actual distance d from the point to the camera,
    // you should calculate the hypotenuse-
    // |code_start| hypot(point.z, point.x);\ |code_end| |br|
    // angle the point made horizontally- |code_start| atan2(point.z,point.x);\ |code_end| |br|
    // angle the point made Vertically- |code_start| atan2(point.z, point.y);\ |code_end| |br|
    // Loop over the entire pointcloud.
    for (auto const point : cloud->points)
    {
      /* Find the coordinates of the highest point */
      if (atan2(point.z, point.y) < min_angle_y)
      {
        min_angle_y = atan2(point.z, point.y);
        lowest_point[0] = point.x;
        lowest_point[1] = point.y;
        lowest_point[2] = point.z;
      }
      /* Find the coordinates of the lowest point */
      else if (atan2(point.z, point.y) > max_angle_y)
      {
        max_angle_y = atan2(point.z, point.y);
        highest_point[0] = point.x;
        highest_point[1] = point.y;
        highest_point[2] = point.z;
      }
    }
    /* Store the center point of cylinder */
    cylinder_params->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
    cylinder_params->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
    cylinder_params->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
    /* Store the height of cylinder */
    cylinder_params->height =
        sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
             pow((lowest_point[2] - highest_point[2]), 2));
    // END_SUB_TUTORIAL
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given a pointcloud extract the ROI defined by the user.
    *
    * @param cloud - Pointcloud whose ROI needs to be extracted.
    */
  void passThroughFilter (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    // min and max values in z axis to keep
    pass.setFilterLimits (0.3, 1.1); //TBD: hard-coded
    pass.filter (*cloud);

    // CLoud Test Output
    std::cout << "/////////////////////" << std::endl;
    std::cout << "Cloud Size: " << cloud->size() << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given a pointcloud apply the voxel grid filter.
    *
    * @param cloud - Pointcloud.
    * @param vg_model - .
    * @param large_leaf_size - .
    * @param small_leaf_size - .
    */
  void voxelGridFilter (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                                                        bool vg_model, 
                                                        float large_leaf_size, 
                                                        float small_leaf_size)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud);
    if (vg_model)
    {
      vg.setLeafSize (large_leaf_size, small_leaf_size, small_leaf_size);
    }
    else
    {
      vg.setLeafSize (small_leaf_size, small_leaf_size, small_leaf_size);
    }
    vg.filter(*cloud);
  }
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given a pointcloud apply the voxel grid filter.
    *
    * @param cloud - Pointcloud.
      @param box - include the maximum/minimum point of the box.
    */
  void cropBoxFilter (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                                           std::vector<double> box)
  {
    pcl::CropBox<pcl::PointXYZRGB> cb;
    cb.setInputCloud(cloud);
    cb.setMin(Eigen::Vector4f(box[0], box[2], box[4], 1.0));
    cb.setMax(Eigen::Vector4f(box[1], box[3], box[5], 1.0));
    cb.setNegative(false);
    cb.filter(*cloud);
  }
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given a pointcloud remove the outliers based on statistic.
    *
    * @param cloud - Pointcloud.
    * @param meanK - Number of nearest neighbors to use for mean 
    *                distance estimation.
    * @param sdm - standard deviation multiplier for the 
    *                 distance threshold calculation.
    */
  void statOutilerRemovalFilter (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                int meanK, float sdm)
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (sdm);
    sor.filter(*cloud);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud and pointer cloud_normals compute the point
    * normals and store in cloud_normals.
    *
    * @param cloud - Pointcloud.
    * @param cloud_normals - The point normals once computer will be stored in
    *                        this.
    */
  void computeNormals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr
      tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch (50); //TBD: hard-coded
    ne.compute (*cloud_normals);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the point normals and point indices, extract the normals for
    *        the indices.
    *
    * @param cloud_normals - Point normals.
    * @param inliers_plane - Indices whose normals need to be extracted.
    */
  void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                      pcl::PointIndices::Ptr inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud and indices of the plane, remove the plannar
    * region from the pointcloud.
    *
    * @param cloud - Pointcloud.
    * @param inliers_plane - Indices representing the plane.
    */
  void removePlaneSurface (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr inliers_plane)
  {
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
    segmentor.setOptimizeCoefficients (true);
    segmentor.setModelType (pcl::SACMODEL_PLANE);
    segmentor.setMethodType (pcl::SAC_RANSAC);
    
    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations (1000); //TBD: hard-coded
    
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold (0.01); //TBD: hard-coded
    segmentor.setInputCloud (cloud);
    
    /* Create the segmentation object for the planar model and set all the
     * parameters
     */
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    segmentor.segment (*inliers_plane, *coefficients_plane);
    
    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud (cloud);
    extract_indices.setIndices (inliers_plane);
    
    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative (true);
    extract_indices.filter (*cloud);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point
    * normals extract the cylinder from the pointcloud and store the cylinder
    * parameters in coefficients_cylinder.
    *
    * @param cloud - Pointcloud whose plane is removed.
    * @param coefficients_cylinder - Cylinder parameters used to define an
    *                                infinite cylinder will be stored here.
    * @param cloud_normals - Point normals corresponding to the plane on which
    *                        cylinder is kept
    */
  void extractCylinder (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        pcl::ModelCoefficients::Ptr coefficients_cylinder,
                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    // Create the segmentation object for cylinder segmentation and set all the
    // parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    segmentor.setOptimizeCoefficients (true);
    segmentor.setModelType (pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType (pcl::SAC_RANSAC);
    
    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight (0.1); //TBD: hard-coded
    
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations (10000); //TBD: hard-coded
    
    // tolerance for variation from model
    segmentor.setDistanceThreshold (0.05); //TBD: hard-coded
    
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits (0, 1); //TBD: hard-coded
    segmentor.setInputCloud (cloud);
    segmentor.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment (*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  void cloudCB (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    FPS_CALC ("cloudCB");
    
    // BEGIN_SUB_TUTORIAL callback
    //
    // Perception Related
    // ^^^^^^^^^^^^^^^^^^
    // First, convert from sensor_msgs to pcl::PointXYZRGB which is needed for
    // most of the processing.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*input, *cloud);

    // Using passthough filter to get region of interest. A passthrough filter
    // just eliminates the point cloud values which do not lie in the user
    // specified range.
    passThroughFilter (cloud);

    if(vg_flag)
    {
      float vg_large_leaf_size = 0.01;
      float vg_small_leaf_size = 0.0075;
      voxelGridFilter(cloud, 0 , vg_large_leaf_size, vg_small_leaf_size);
      std::cout << "Cloud Size After VG: " << cloud->size() << std::endl;
    }

    if(cb_flag)
    {
      double est_x = -0.05;
      double est_y = 0.16;
      double est_z = 0.92;
      double cdf_pred = 0.25;
      std::vector<double> cbfbox = {est_x - cdf_pred, est_x + cdf_pred,
                                est_y - cdf_pred, est_y + cdf_pred,
                                est_z - cdf_pred, est_z + cdf_pred};
      cropBoxFilter(cloud,cbfbox);
      std::cout << "Cloud Size After CBF: " << cloud->size() << std::endl;
    }

    if(sor_flag)
    {
      int meanK = 50;
      float sdm = 1.0;
      statOutilerRemovalFilter(cloud,meanK,sdm);
      std::cout << "Cloud Size After SOR: " << cloud->size() << std::endl;
    }
    
    // Declare normals and call function to compute point normals.
    pcl::PointCloud<pcl::Normal>::Ptr
      cloud_normals (new pcl::PointCloud<pcl::Normal>);
    computeNormals (cloud, cloud_normals);
    
    // inliers_plane will hold the indices of the point cloud that correspond
    // to a plane.
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    
    // Detect and eliminate the plane on which the cylinder is kept to ease the
    // process of finding the cylinder.
    removePlaneSurface (cloud, inliers_plane);
    
    // We had calculated the point normals in a previous call to computeNormals,
    // now we will be extracting the normals that correspond to the plane on
    // which cylinder lies.
    // It will be used to extract the cylinder.
    extractNormals (cloud_normals, inliers_plane);
    
    // ModelCoefficients will hold the parameters using which we can define a
    // cylinder of infinite length.
    // It has a public attribute |code_start| values\ |code_end| of type
    // |code_start| std::vector< float >\ |code_end|\
    // .
    // |br|
    // |code_start| Values[0-2]\ |code_end| hold a point on the center line of
    // the cylinder. |br|
    // |code_start| Values[3-5]\ |code_end| hold direction vector of the z-axis.
    // |br|
    // |code_start| Values[6]\ |code_end| is the radius of the cylinder.
    pcl::ModelCoefficients::Ptr
      coefficients_cylinder (new pcl::ModelCoefficients);
    
    // Extract the cylinder using SACSegmentation.
    extractCylinder (cloud, coefficients_cylinder, cloud_normals);
    
    // END_SUB_TUTORIAL
    if (cloud->points.empty())
    {
      ROS_ERROR_STREAM_NAMED ("cylinder_segment",
                              "Can't find the cylindrical component.");
      return;
    }
    // Update point cloud
    if (update_sign == 1)
    {
      std::vector<std::string> object_ids;
      object_ids.push_back("cylinder");
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      planning_scene_interface.removeCollisionObjects(object_ids);
      update_sign = 0;
      points_not_found = true;
    }
    
    if (points_not_found)
    {
      // BEGIN_TUTORIAL
      // CALL_SUB_TUTORIAL callback
      //
      // Storing Relevant Cylinder Values
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // The information that we have in |code_start| coefficients_cylinder\
      // |code_end| is not enough to define our cylinder.
      // It does not have the actual location of the cylinder nor the actual
      // height. |br|
      // We define a struct to hold the parameters that are actually needed for
      // defining a collision object completely.
      // |br|
      // CALL_SUB_TUTORIAL param_struct
      
      cylinder_params = new AddCylinderParams();
      // Store the radius of the cylinder.
      cylinder_params->radius = coefficients_cylinder->values[6];
      // Store direction vector of z-axis of cylinder.
      cylinder_params->direction_vec[0] = coefficients_cylinder->values[3];
      cylinder_params->direction_vec[1] = coefficients_cylinder->values[4];
      cylinder_params->direction_vec[2] = coefficients_cylinder->values[5];
      
      // Extracting Location and Height
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // Compute the center point of the cylinder using standard geometry
      extractLocationHeight(cloud);
      // CALL_SUB_TUTORIAL extract_location_height
      // Use the parameters extracted to add the cylinder to the planning scene
      // as a collision object.
      addCylinder();
      // CALL_SUB_TUTORIAL add_cylinder
      // END_TUTORIAL
      points_not_found = false;
    }
    /* updating the position continuously */
    else
    {
      // Update the cylinder pose when being grasped
      if (grasped)
      {
        std::cout << "Cylinder is not grasped" << std::endl;
        // Listen the Transform -> T(0,ee)
        tf::TransformListener listener;
        tf::StampedTransform Trans_E2B;
        // According to http://wiki.ros.org/tf/Tutorials/
        try{
          transformListener.waitForTransform("/panda_link0", "/panda_hand",
                                             ros::Time(0), ros::Duration(2.0));
          transformListener.lookupTransform("/panda_link0", "/panda_hand",
                                            ros::Time(0), Trans_E2B);
        }catch(tf::TransformException& e){
          ROS_ERROR("%s", e.what());
          ros::Duration(1.0).sleep();
        }
        if (start_grasp)
        {
          std::cout << "Cylinder Finish grasping" << std::endl;
          // T(0, cy)
          tf::StampedTransform  Trans_E2B_inv; 
          Trans_E2B_inv.setData(Trans_E2B.inverse());
          Trans_C2E.setData(Trans_E2B_inv * Trans_C2B);
          start_grasp = false;
        }
        update_publish_cylinder_pose(Trans_E2B,Trans_C2E);
      }
      else
      {
        std::cout << "Cylinder is being grasped" << std::endl;
        start_grasp = true;
        tf::StampedTransform  Trans_E2B_inv;
        // Listen the Transform -> T(0,ee)
        tf::TransformListener listener;
        tf::StampedTransform Trans_E2B;
        // According to http://wiki.ros.org/tf/Tutorials/
        try{
          transformListener.waitForTransform("/panda_link0", "/panda_hand",
                                             ros::Time(0), ros::Duration(2.0));
          transformListener.lookupTransform("/panda_link0", "/panda_hand",
                                            ros::Time(0), Trans_E2B);
        }catch(tf::TransformException& e){
          ROS_ERROR("%s", e.what());
          ros::Duration(1.0).sleep();
        }
        Trans_E2B_inv.setData(Trans_E2B.inverse());
        Trans_C2E.setData(Trans_E2B_inv * Trans_C2B);
        update_publish_cylinder_pose(Trans_E2B,Trans_C2E);
      }
    }
  }

  void update_publish_cylinder_pose(tf::StampedTransform Trans_E2B,
                                    tf::StampedTransform Trans_C2E)
  {
      // T(0, cy) = T(0, ee) * T(ee, cy)
      Trans_C2B.setData(Trans_E2B * Trans_C2E);
      geometry_msgs::PoseStamped cylinder_msg;

      //SetData
      cylinder_msg.pose.position.x = Trans_C2B.getOrigin().getX();
      cylinder_msg.pose.position.y = Trans_C2B.getOrigin().getY();
      cylinder_msg.pose.position.z = Trans_C2B.getOrigin().getZ();
      cylinder_msg.pose.orientation.x = Trans_C2B.getRotation().getX();
      cylinder_msg.pose.orientation.y = Trans_C2B.getRotation().getY();
      cylinder_msg.pose.orientation.z = Trans_C2B.getRotation().getZ();
      cylinder_msg.pose.orientation.w = Trans_C2B.getRotation().getW();
      //Output test
      Output_cylinder_states(cylinder_msg);
      //Publish 
      pub_cylinder_pose.publish(cylinder_msg);
  }

  void Output_cylinder_states (geometry_msgs::PoseStamped cylinder_msg)
  {
    std::cout << "--------------" << std::endl;
     std::cout << "Cylinder Position: " << std::endl
              << "X: "<< cylinder_msg.pose.position.x<< std::endl
              << "Y: "<< cylinder_msg.pose.position.y<< std::endl
              << "Z: "<< cylinder_msg.pose.position.z<< std::endl;
    std::cout << "--------------" << std::endl;
  }
    
  void set_grasp_status (const std_msgs::Bool& input)
  {
    grasped = input.data;
  }

  // Cloud Filter Helper Function
  // Switch VGF
  void switch_vg_flag (const std_msgs::Float64& input)
  {
    if (vg_flag)
    {
      vg_flag = false;
      std::cout << "VG is OFF" << std::endl;
    }
    else
    {
      vg_flag = true;
      std::cout << "VG is ON" << std::endl;
    }
    // Update segmentation/filtering
    update_sign = update_sign <= 1 ? 1 : update_sign;
  }
  // Switch CBF
  void switch_cb_flag (const std_msgs::Float64& input)
  {
    if (cb_flag)
    {
      cb_flag = false;
      std::cout << "CB is OFF" << std::endl;
    }
    else
    {
      cb_flag = true;
      std::cout << "CB is ON" << std::endl;
    }
    // Update segmentation/filtering
    update_sign = update_sign <= 1 ? 1 : update_sign;
  }
  // Switch SOR
  void switch_sor_flag (const std_msgs::Float64& input)
  {
    if(sor_flag)
    {
      sor_flag = false;
      std::cout << "SOR is OFF" << std::endl;
    }
    else
    {
      sor_flag = true;
      std::cout << "SOR is ON" << std::endl;
    }
    update_sign = update_sign <= 1 ? 1 : update_sign;
  }

private:
  // BEGIN_SUB_TUTORIAL param_struct
  // There are 4 fields and a total of 7 parameters used to define this.
  struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector towards the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };
  
  // Declare a variable of type AddCylinderParams and store relevant values
  // from ModelCoefficients.
  AddCylinderParams* cylinder_params;
  // END_SUB_TUTORIAL

  // Publishers
  ros::Publisher pub_cylinder_pose;
  ros::Publisher pub_cylinder_radius;
  ros::Publisher pub_cylinder_height;
  // Listener
  tf::TransformListener transformListener;

  // Flag
  // grasping flag
  bool grasped = true;
  // start grasp flag
  bool start_grasp = true;
  // Update signal
  int update_sign = 0;
  // VG Flag
  bool vg_flag = false;
  // CB Flag
  bool cb_flag = false;
  // SOR Flag
  bool sor_flag = false;

  // Transforms
  // T(0,cylinder) base to cylinder. At Begaining.
  tf::StampedTransform Trans_C2B;
  // T(ee,cylinder) end-effector to cylinder. When Grasping.
  tf::StampedTransform Trans_C2E;

  bool points_not_found = true;
};



int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cylinder_segment");
  
  // Start the segmentor
  CylinderSegment ();
}
