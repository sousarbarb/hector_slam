#pragma once

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Header.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// Hector SLAM
#include "PoseInfoContainer.h"
#include "scan/DataPointContainer.h"
#include "slam_main/HectorSlamProcessor.h"

// Forward declarations
class HectorDrawings;
class HectorDebugInfoProvider;

class HectorMappingROS1API
{
 public:

  HectorMappingROS1API();
  virtual ~HectorMappingROS1API();

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  void rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan,
                                   hectorslam::DataContainer& dataContainer,
                                   float scaleToMap);
  void rosPointCloudToDataContainer(
      const sensor_msgs::PointCloud& pointCloud,
      const geometry_msgs::TransformStamped& laserTransform,
      hectorslam::DataContainer& dataContainer, float scaleToMap);

  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void toggleMappingPause(bool pause);
  void resetPose(const geometry_msgs::Pose& pose);

 protected:

  virtual void pubOdom() = 0;
  virtual void pubPose() = 0;
  virtual void pubMap(const std_msgs::Header& header) = 0;
  virtual void pubScanPointCloud(const sensor_msgs::PointCloud&) = 0;
  virtual void pubMapOdomTransform() = 0;
  virtual void pubMapScanMatchTransform() = 0;

 protected:

  HectorDebugInfoProvider* debugInfoProvider;
  HectorDrawings* hectorDrawings;

  int lastGetMapUpdateIndex;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  tf2_ros::Buffer tf2_buffer_;

  laser_geometry::LaserProjection projector_;

  tf2::Transform map_to_odom_;

  hectorslam::HectorSlamProcessor* slamProcessor;
  hectorslam::DataContainer laserScanContainer;

  PoseInfoContainer poseInfoContainer_;

  sensor_msgs::PointCloud laser_point_cloud_;

  ros::Time lastMapPublishTime_;

  bool initial_pose_set_;
  Eigen::Vector3f initial_pose_;

  bool pause_scan_processing_;

  //-----------------------------------------------------------
  // Parameters

  std::string p_base_frame_;
  std::string p_map_frame_;
  std::string p_odom_frame_;

  // Parameters related to publishing the scanmatcher pose directly via tf
  bool p_pub_map_scanmatch_transform_;
  std::string p_tf_map_scanmatch_transform_frame_name_;

  std::string p_scan_topic_;
  std::string p_sys_msg_topic_;

  std::string p_pose_update_topic_;
  std::string p_twist_update_topic_;

  bool p_pub_drawings;
  bool p_pub_debug_output_;
  bool p_pub_map_odom_transform_;
  bool p_pub_odometry_;
  bool p_advertise_map_service_;
  int p_scan_subscriber_queue_size_;

  double p_update_factor_free_;
  double p_update_factor_occupied_;
  double p_map_update_distance_threshold_;
  double p_map_update_angle_threshold_;

  double p_map_resolution_;
  int p_map_size_;
  double p_map_start_x_;
  double p_map_start_y_;
  int p_map_multi_res_levels_;

  double p_map_pub_period_;

  bool p_use_tf_scan_transformation_;
  bool p_use_tf_pose_start_estimate_;
  bool p_map_with_known_poses_;
  bool p_timing_output_;

  float p_sqr_laser_min_dist_;
  float p_sqr_laser_max_dist_;
  float p_laser_z_min_value_;
  float p_laser_z_max_value_;

};  // class HectorMappingROS1API
