#include "HectorMappingROS1Offline.h"

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>

// ROS
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_msgs/TFMessage.h>

#include "HectorMapMutex.h"

HectorMappingROS1Offline::HectorMappingROS1Offline(const ParamOffline& param)
    : HectorMappingROS1API::HectorMappingROS1API(), param_offline_(param)
{
  ros::Time::init();

  tf2_buffer_.setUsingDedicatedThread(true);

  // Print offline parametrization
  std::stringstream str;

  for (const std::string& bag_filename : param_offline_.bags)
  {
    str << "- " << bag_filename << std::endl;
  }

  ROS_INFO("[%s] bag files:\n%s", ros::this_node::getName().c_str(),
           str.str().c_str());
  ROS_INFO("[%s] initial pose topic: %s", ros::this_node::getName().c_str(),
           param_offline_.initial_pose_topic.c_str());
  ROS_INFO("[%s] scan         topic: %s", ros::this_node::getName().c_str(),
           param_offline_.scan_topic.c_str());

  // Log file processing
  if (param_offline_.enable_log)
  {
    if (param_offline_.log_filename.empty())
    {
      throw std::runtime_error(
          "HectorMappingROS1Offline::HectorMappingROS1Offline | empty filename "
          "when log enabled");
    }

    std::string log_file_pose;

    try
    {
      std::filesystem::path log_file_path(param_offline_.log_filename);

      log_file_pose = log_file_path.stem().string() + "_hector_pose" +
                      log_file_path.extension().string();

      ROS_INFO("[%s] log file          : %s", ros::this_node::getName().c_str(),
               log_file_pose.c_str());
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      throw std::runtime_error(
          "HectorMappingROS1Offline::HectorMappingROS1Offline | Error "
          "resolving paths for log files");
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "HectorMappingROS1Offline::HectorMappingROS1Offline | Error when "
          "processing paths for log files");
    }
    catch (...)
    {
      throw std::runtime_error(
          "HectorMappingROS1Offline::HectorMappingROS1Offline | Unknown error "
          "when processing paths for log files");
    }

    validateAndCreatePath(log_file_pose);

    try
    {
      log_file_pose_ = std::ofstream(log_file_pose);

      if (!log_file_pose_.is_open())
      {
        throw std::runtime_error(
            "HectorMappingROS1Offline::HectorMappingROS1Offline | file (" +
            param_offline_.log_filename + ") for pose data not opened");
      }
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "HectorMappingROS1Offline::HectorMappingROS1Offline | error when "
          "opening the log file (" +
          log_file_pose + "): " + e.what());
    }
  }
  else
  {
    ROS_INFO("[%s] log file          : not enabled",
             ros::this_node::getName().c_str());
  }

  std::cout << std::endl;

  // ROS API
  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer_.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr("map");

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer_[i];
    tmp.mapPublisher_ =
        nh_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ =
        nh_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if (i == 0)
    {
      mapPubContainer_[i].mapMetadataPublisher_.publish(
          mapPubContainer_[i].map_.map.info);
    }
  }

  initial_pose_filter_ = std::make_unique<
      tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>>(
      tf2_buffer_, p_map_frame_, 10, nh_priv_);

  initial_pose_filter_->registerCallback(
      &HectorMappingROS1API::initialPoseCallback,
      static_cast<HectorMappingROS1API*>(this));
}

HectorMappingROS1Offline::~HectorMappingROS1Offline()
{
  if (param_offline_.enable_log && log_file_pose_.is_open())
  {
    log_file_pose_.close();
  }

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    ROS_INFO("[%s] Closing %s", ros::this_node::getName().c_str(),
             bag->getFileName().c_str());

    bag->close();
  }
}

void HectorMappingROS1Offline::run()
{
  std::cout << std::endl;

  auto start = std::chrono::high_resolution_clock::now();

  for (const std::string& filename : param_offline_.bags)
  {
    ROS_INFO("[%s] Opening %s", ros::this_node::getName().c_str(),
             filename.c_str());

    try
    {
      std::shared_ptr<rosbag::Bag> bag = std::make_shared<rosbag::Bag>();

      bag->open(filename, rosbag::bagmode::Read);

      bags_.push_back(bag);
    }
    catch (rosbag::BagException& e)
    {
      std::stringstream error;

      error << "Error when opening the ROS bag file (filename: " << filename
            << "; error: " << e.what() << ")";

      throw std::runtime_error(error.str());
    }
  }

  rosbag::View full_view;

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    full_view.addQuery(*bag);
  }

  const ros::Time full_initial_time = full_view.getBeginTime();
  const ros::Time initial_time =
      full_initial_time + ros::Duration(param_offline_.time_start);
  ros::Time finish_time = ros::TIME_MAX;

  ROS_INFO("[%s] Start  time (s): %.9lf", ros::this_node::getName().c_str(),
           initial_time.toSec());

  if (param_offline_.has_duration)
  {
    finish_time = initial_time + ros::Duration(param_offline_.time_duration);

    ROS_INFO("[%s] Finish time (s): %.9lf", ros::this_node::getName().c_str(),
             finish_time.toSec());
    ROS_INFO("[%s] Total  time (s): %.9lf\n", ros::this_node::getName().c_str(),
             ros::Duration(finish_time - initial_time).toSec());
  }
  else
  {
    ROS_INFO("[%s] Finish time (s): %.9lf (end of the bags)",
             ros::this_node::getName().c_str(), full_view.getEndTime().toSec());
    ROS_INFO("[%s] Total  time (s): %.9lf\n", ros::this_node::getName().c_str(),
             ros::Duration(full_view.getEndTime() - initial_time).toSec());
  }

  rosbag::View view;

  for (const std::shared_ptr<rosbag::Bag>& bag : bags_)
  {
    view.addQuery(*bag, initial_time, finish_time);
  }

  for (rosbag::MessageInstance const& msg : view)
  {
    if (msg.instantiate<sensor_msgs::LaserScan>() != nullptr)
    {
      if (msg.getTopic() != param_offline_.scan_topic)
      {
        continue;
      }

      sensor_msgs::LaserScanPtr laser_msg =
          msg.instantiate<sensor_msgs::LaserScan>();

      scanCallback(laser_msg);
    }
    else if (msg.instantiate<tf2_msgs::TFMessage>() != nullptr)
    {
      tf2_msgs::TFMessagePtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();

      bool is_static = (msg.getTopic().compare("/tf_static") == 0);

      for (const geometry_msgs::TransformStamped& transf : tf_msg->transforms)
      {
        tf2_buffer_.setTransform(transf, ros::this_node::getName(), is_static);
      }
    }
    else if (msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>() !=
             nullptr)
    {
      if (msg.getTopic() != param_offline_.initial_pose_topic)
      {
        continue;
      }

      geometry_msgs::PoseWithCovarianceStampedPtr initial_pose_msg =
          msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>();

      initial_pose_filter_->add(initial_pose_msg);
    }

    ros::spinOnce();
  }

  auto end = std::chrono::high_resolution_clock::now();

  ROS_INFO(
      "\n\n"
      "[%s] Finished processing the ROS bags.\n"
      "Elapsed time (s): %.3lf\n"
      "ros::spin to allow rosrun map_server map_saver OR rviz "
      "visualization.",
      ros::this_node::getName().c_str(),
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
              .count() *
          1e-6);

  ros::spin();
}

void HectorMappingROS1Offline::setServiceGetMapData(
    nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin(gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength() / 1000 * 0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

void HectorMappingROS1Offline::pubPose()
{
  if (!param_offline_.enable_log)
  {
    return;
  }

  const geometry_msgs::PoseStamped& mpose = poseInfoContainer_.getPoseStamped();

  tf2::Quaternion mpose_q;
  mpose_q.setRPY(0, 0, tf2::getYaw(mpose.pose.orientation));

  try
  {
    log_file_pose_ << std::fixed << std::setprecision(9)
                   << mpose.header.stamp.toSec() << " " << mpose.pose.position.x
                   << " " << mpose.pose.position.y << " " << 0 << " "
                   << mpose_q.x() << " " << mpose_q.y() << " " << mpose_q.z()
                   << " " << mpose_q.w() << std::endl;
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "HectorMappingROS1Offline::pubPose | error when logging the robot data "
        "(" +
        std::string(e.what()) + ")");
  }
}

void HectorMappingROS1Offline::pubMap(const std_msgs::Header& header)
{
  nav_msgs::GetMap::Response& map_(mapPubContainer_[0].map_);

  // only update map if it changed
  if (lastGetMapUpdateIndex != slamProcessor->getGridMap(0).getUpdateIndex())
  {
    int sizeX = slamProcessor->getGridMap(0).getSizeX();
    int sizeY = slamProcessor->getGridMap(0).getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    // std::vector contents are guaranteed to be contiguous, use memset to set
    // all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    for (int i = 0; i < size; ++i)
    {
      if (slamProcessor->getGridMap(0).isFree(i))
      {
        data[i] = 0;
      }
      else if (slamProcessor->getGridMap(0).isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = slamProcessor->getGridMap(0).getUpdateIndex();
  }

  map_.map.header.stamp = header.stamp;

  mapPubContainer_[0].mapPublisher_.publish(map_.map);
}

void HectorMappingROS1Offline::validateAndCreatePath(
    const std::string& file_path)
{
  try
  {
    std::filesystem::path path(file_path);
    std::filesystem::path directory = path.parent_path();
    std::filesystem::path curr_directory = std::filesystem::current_path();

    ROS_INFO("[%s] Current path directory (%s)",
             ros::this_node::getName().c_str(),
             curr_directory.string().c_str());

    if (directory.empty())
    {
      ROS_INFO("[%s] Using current path directory.",
               ros::this_node::getName().c_str());
      return;
    }

    if (std::filesystem::exists(directory))
    {
      if (std::filesystem::is_directory(directory))
      {
        ROS_INFO("[%s] Directory (%s) exists",
                 ros::this_node::getName().c_str(), directory.string().c_str());
        return;
      }
      else
      {
        ROS_ERROR("[%s] Path (%s) exists but is not a directory",
                  ros::this_node::getName().c_str(),
                  directory.string().c_str());

        throw std::runtime_error(
            "HectorMappingROS1Offline::validateAndCreatePath | Path (" +
            directory.string() + ") exists but is not a directory");
      }
    }
    else
    {
      ROS_INFO("[%s] Directory doesn't exist. Creating: %s",
               ros::this_node::getName().c_str(), directory.string().c_str());

      if (std::filesystem::create_directory(directory))
      {
        ROS_INFO("[%s] Directory created successfully.",
                 ros::this_node::getName().c_str());
        return;
      }
      else
      {
        ROS_ERROR("[%s] Failed to create directory.",
                  ros::this_node::getName().c_str());

        throw std::runtime_error(
            "HectorMappingROS1Offline::validateAndCreatePath | Failed to "
            "create "
            "directory (" +
            directory.string() + ")");
      }
    }
  }
  catch (const std::filesystem::filesystem_error& e)
  {
    throw std::runtime_error(
        "HectorMappingROS1Offline::validateAndCreatePath | Error resolving "
        "path (" +
        file_path + "): " + e.what());
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "HectorMappingROS1Offline::validateAndCreatePath | Error when "
        "processing path (" +
        file_path + "): " + e.what());
  }
}
