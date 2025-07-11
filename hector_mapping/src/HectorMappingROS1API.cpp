#include "HectorMappingROS1API.h"

#include <chrono>

#include "HectorDebugInfoProvider.h"
#include "HectorDrawings.h"

HectorMappingROS1API::HectorMappingROS1API()
    : debugInfoProvider(0),
      hectorDrawings(0),
      lastGetMapUpdateIndex(-100),
      initial_pose_set_(false),
      pause_scan_processing_(false),
      nh_priv_("~"),
      tf2_buffer_(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME), false)
{
  std::string mapTopic_ = "map";

  nh_priv_.param("pub_drawings", p_pub_drawings, false);
  nh_priv_.param("pub_debug_output", p_pub_debug_output_, false);
  nh_priv_.param("pub_map_odom_transform", p_pub_map_odom_transform_, true);
  nh_priv_.param("pub_odometry", p_pub_odometry_, false);
  nh_priv_.param("advertise_map_service", p_advertise_map_service_, true);
  nh_priv_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_,
                 5);

  nh_priv_.param("map_resolution", p_map_resolution_, 0.025);
  nh_priv_.param("map_size", p_map_size_, 1024);
  nh_priv_.param("map_start_x", p_map_start_x_, 0.5);
  nh_priv_.param("map_start_y", p_map_start_y_, 0.5);
  nh_priv_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

  nh_priv_.param("update_factor_free", p_update_factor_free_, 0.4);
  nh_priv_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

  nh_priv_.param("map_update_distance_thresh", p_map_update_distance_threshold_,
                 0.4);
  nh_priv_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

  nh_priv_.param("scan_topic", p_scan_topic_, std::string("scan"));
  nh_priv_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
  nh_priv_.param("pose_update_topic", p_pose_update_topic_,
                 std::string("poseupdate"));

  nh_priv_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,
                 true);
  nh_priv_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,
                 false);
  nh_priv_.param("map_with_known_poses", p_map_with_known_poses_, false);

  nh_priv_.param("base_frame", p_base_frame_, std::string("base_link"));
  nh_priv_.param("map_frame", p_map_frame_, std::string("map"));
  nh_priv_.param("odom_frame", p_odom_frame_, std::string("odom"));

  nh_priv_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,
                 true);
  nh_priv_.param("tf_map_scanmatch_transform_frame_name",
                 p_tf_map_scanmatch_transform_frame_name_,
                 std::string("scanmatcher_frame"));

  nh_priv_.param("output_timing", p_timing_output_, false);

  nh_priv_.param("map_pub_period", p_map_pub_period_, 2.0);

  double tmp = 0.0;
  nh_priv_.param("laser_min_dist", tmp, 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp * tmp);

  nh_priv_.param("laser_max_dist", tmp, 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp * tmp);

  nh_priv_.param("laser_z_min_value", tmp, -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  nh_priv_.param("laser_z_max_value", tmp, 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  slamProcessor = new hectorslam::HectorSlamProcessor(
      static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_,
      Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_,
      hectorDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s",
           p_use_tf_scan_transformation_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_pub_map_odom_transform_: %s",
           p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d",
           p_scan_subscriber_queue_size_);
  ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ",
           p_map_update_distance_threshold_);
  ROS_INFO("HectorSM p_map_update_angle_threshold_: %f",
           p_map_update_angle_threshold_);
  ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  map_to_odom_.setIdentity();

  lastMapPublishTime_ = ros::Time(0, 0);
}

HectorMappingROS1API::~HectorMappingROS1API()
{
  delete slamProcessor;

  if (hectorDrawings) delete hectorDrawings;

  if (debugInfoProvider) delete debugInfoProvider;
}

void HectorMappingROS1API::scanCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if (pause_scan_processing_)
  {
    return;
  }

  if (hectorDrawings)
  {
    hectorDrawings->setTime(scan->header.stamp);
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  if (!p_use_tf_scan_transformation_)
  {
    // If we are not using the tf tree to find the transform between the base
    // frame and laser frame, then just convert the laser scan to our data
    // container and process the update based on our last pose estimate
    this->rosLaserScanToDataContainer(*scan, laserScanContainer,
                                      slamProcessor->getScaleToMap());
    slamProcessor->update(laserScanContainer,
                          slamProcessor->getLastScanMatchPose());
  }
  else
  {
    // If we are using the tf tree to find the transform between the base frame
    // and laser frame, let's get that transform
    geometry_msgs::TransformStamped laser_transform;

    try
    {
      laser_transform = tf2_buffer_.lookupTransform(
          p_base_frame_, scan->header.frame_id, ros::Time(0));
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN(
          "lookupTransform %s to %s timed out. Could not transform laser scan "
          "into base_frame.",
          p_base_frame_.c_str(), scan->header.frame_id.c_str());
      return;
    }

    // Convert the laser scan to point cloud
    projector_.projectLaser(*scan, laser_point_cloud_, 30.0);

    // Publish the point cloud if there are any subscribers
    pubScanPointCloud(laser_point_cloud_);

    // Convert the point cloud to our data container
    this->rosPointCloudToDataContainer(laser_point_cloud_, laser_transform,
                                       laserScanContainer,
                                       slamProcessor->getScaleToMap());

    // Now let's choose the initial pose estimate for our slam process update
    Eigen::Vector3f start_estimate(Eigen::Vector3f::Zero());
    if (initial_pose_set_)
    {
      // User has requested a pose reset
      initial_pose_set_ = false;
      start_estimate = initial_pose_;
    }
    else if (p_use_tf_pose_start_estimate_)
    {
      // Initial pose estimate comes from the tf tree
      try
      {
        geometry_msgs::TransformStamped stamped_pose =
            tf2_buffer_.lookupTransform(p_map_frame_, p_base_frame_,
                                        scan->header.stamp);

        const double yaw = tf2::getYaw(stamped_pose.transform.rotation);

        start_estimate =
            Eigen::Vector3f(stamped_pose.transform.translation.x,
                            stamped_pose.transform.translation.y, yaw);
      }
      catch (const tf2::TransformException& e)
      {
        ROS_ERROR("Transform from %s to %s failed\n", p_map_frame_.c_str(),
                  p_base_frame_.c_str());
        start_estimate = slamProcessor->getLastScanMatchPose();
      }
    }
    else
    {
      // If none of the above, the initial pose is simply the last estimated
      // pose
      start_estimate = slamProcessor->getLastScanMatchPose();
    }

    // If "p_map_with_known_poses_" is enabled, we assume that start_estimate is
    // precise and doesn't need to be refined
    if (p_map_with_known_poses_)
    {
      slamProcessor->update(laserScanContainer, start_estimate, true);
    }
    else
    {
      slamProcessor->update(laserScanContainer, start_estimate);
    }
  }

  // If the debug flag "p_timing_output_" is enabled, print how long this last
  // iteration took
  if (p_timing_output_)
  {
    int64_t duration =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time)
            .count();
    ROS_INFO("HectorSLAM Iter took: %.3f milliseconds", (duration * 1e-3));
  }

  // If we're just building a map with known poses, we're finished now. Code
  // below this point publishes the localization results.
  if (p_map_with_known_poses_)
  {
    return;
  }

  poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(),
                            slamProcessor->getLastScanMatchCovariance(),
                            scan->header.stamp, p_map_frame_);

  pubOdom();
  pubPose();

  if (ros::Duration(scan->header.stamp - lastMapPublishTime_).toSec() >
      1 / p_map_pub_period_)
  {
    pubMap(scan->header);
    lastMapPublishTime_ = scan->header.stamp;
  }

  // Publish the map->odom transform if enabled
  pubMapOdomTransform();

  // Publish the transform from map to estimated pose (if enabled)
  pubMapScanMatchTransform();
}

void HectorMappingROS1API::rosLaserScanToDataContainer(
    const sensor_msgs::LaserScan& scan,
    hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ((dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

void HectorMappingROS1API::rosPointCloudToDataContainer(
    const sensor_msgs::PointCloud& pointCloud,
    const geometry_msgs::TransformStamped& laserTransform,
    hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();
  // ROS_INFO("size: %d", size);

  tf2::Transform laser_tf(tf2::Quaternion(laserTransform.transform.rotation.x,
                                          laserTransform.transform.rotation.y,
                                          laserTransform.transform.rotation.z,
                                          laserTransform.transform.rotation.w),
                          tf2::Vector3(laserTransform.transform.translation.x,
                                       laserTransform.transform.translation.y,
                                       laserTransform.transform.translation.z));

  dataContainer.clear();

  tf2::Vector3 laserPos(laser_tf.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y()) *
                         scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {
    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x * currPoint.x + currPoint.y * currPoint.y;

    if ((dist_sqr > p_sqr_laser_min_dist_) &&
        (dist_sqr < p_sqr_laser_max_dist_))
    {
      if ((currPoint.x < 0.0f) && (dist_sqr < 0.50f))
      {
        continue;
      }

      tf2::Vector3 pointPosBaseFrame(
          laser_tf * tf2::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ &&
          pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(
            Eigen::Vector2f(pointPosBaseFrame.x(), pointPosBaseFrame.y()) *
            scaleToMap);
      }
    }
  }
}

void HectorMappingROS1API::initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  this->resetPose(msg->pose.pose);
}

void HectorMappingROS1API::toggleMappingPause(bool pause)
{
  // Pause/unpause
  if (pause && !pause_scan_processing_)
  {
    ROS_INFO("[HectorSM]: Mapping paused");
  }
  else if (!pause && pause_scan_processing_)
  {
    ROS_INFO("[HectorSM]: Mapping no longer paused");
  }
  pause_scan_processing_ = pause;
}

void HectorMappingROS1API::resetPose(const geometry_msgs::Pose& pose)
{
  initial_pose_set_ = true;
  initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y,
                                  util::getYawFromQuat(pose.orientation));
  ROS_INFO(
      "[HectorSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
      initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}
