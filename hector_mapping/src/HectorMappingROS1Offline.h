#pragma once

#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

// ROS
#include <nav_msgs/GetMap.h>
#include <rosbag/bag.h>
#include <tf2_ros/message_filter.h>

// ROS API
#include "HectorMappingROS1API.h"

class MapPublisherContainer
{
 public:

  ros::Publisher mapPublisher_;
  ros::Publisher mapMetadataPublisher_;
  nav_msgs::GetMap::Response map_;
  ros::ServiceServer dynamicMapServiceServer_;
};

class HectorMappingROS1Offline : HectorMappingROS1API
{
 public:

  struct ParamOffline
  {
    std::vector<std::string> bags;   //!< set of ROS bag files to process
    std::string initial_pose_topic;  //!< initial pose topic name
    std::string scan_topic;          //!< scan topic name for 2D laser data
    bool has_duration;         //!< duration from the start time set in options
    double time_start;         //!< start time (s) into the bag files
    double time_duration;      //!< duration (s) to only process from the bags
    bool enable_log;           //!< enable log of robot data (pose) into TUM
    std::string log_filename;  //!< log filename
  };  // struct HectorMappingROS1Offline::ParamOffline

 public:

  HectorMappingROS1Offline(const ParamOffline& param);
  virtual ~HectorMappingROS1Offline();

  void run();

 protected:

  virtual void pubOdom() final {}
  virtual void pubPose() final;
  virtual void pubMap(const std_msgs::Header& header) final;
  virtual void pubScanPointCloud(const sensor_msgs::PointCloud&) {}
  virtual void pubMapOdomTransform() {}
  virtual void pubMapScanMatchTransform() {}

  void setServiceGetMapData(nav_msgs::GetMap::Response& map_,
                            const hectorslam::GridMap& gridMap);

  virtual void setupTerminal();
  virtual void restoreTerminal();
  virtual void printTime(const ros::Time& t, const ros::Duration& duration,
                         const ros::Duration& bag_length) const;

  virtual char readTerminalKey() const;

 private:

  HectorMappingROS1Offline() = delete;

  void validateAndCreatePath(const std::string& file_path);

 protected:

  ParamOffline param_offline_;

  bool paused_ = false;

  bool terminal_modified_ = false;

  termios orig_flags_;

  std::unique_ptr<
      tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>>
      initial_pose_filter_;

  std::vector<MapPublisherContainer> mapPubContainer_;

  std::vector<std::shared_ptr<rosbag::Bag>> bags_;

  std::ofstream log_file_pose_;
};  // class HectorMappingROS1Offline : HectorMappingROS1API
