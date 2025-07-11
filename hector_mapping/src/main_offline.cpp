#include <iostream>

// Boost
#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>

#include "HectorMappingROS1Offline.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hector_slam_offline",
            ros::init_options::NoSigintHandler);

  HectorMappingROS1Offline::ParamOffline param;

  boost::program_options::options_description opts_desc(
      "Usage: rosrun hector_mapping hector_slam_offline "
      "-b BAGFILE1 [BAGFILE2 BAGFILE3 ...]\n\nOptions");

  opts_desc.add_options()("help,h", "Display this information.")(
      "bags,b",
      boost::program_options::value<std::vector<std::string>>(&param.bags)
          ->required()
          ->multitoken()
          ->value_name("BAGFILE"),
      "ROS bag files to process")(
      "initialposetopic",
      boost::program_options::value<std::string>(&param.initial_pose_topic)
          ->default_value("/initial_pose")
          ->value_name("TOPIC"),
      "Topic name for the robot's odometry data (used for initialization)")(
      "scantopic",
      boost::program_options::value<std::string>(&param.scan_topic)
          ->default_value("/scan")
          ->value_name("TOPIC"),
      "Topic name for the 2D laser scanner data")(
      "start,s",
      boost::program_options::value<double>(&param.time_start)
          ->default_value(0.0)
          ->value_name("SEC"),
      "start SEC seconds into the bag files")(
      "duration,d",
      boost::program_options::value<double>(&param.time_duration)
          ->value_name("SEC"),
      "play only SEC seconds from the bag files")(
      "log",
      boost::program_options::value<std::string>(&param.log_filename)
          ->value_name("FILENAME"),
      "log the robot estimated data (laser pose) into TUM files");

  /* boost::program_options::positional_options_description opts_pos;
  opts_pos.add("bags", -1); */

  boost::program_options::variables_map opts_vm;

  try
  {
    boost::program_options::store(
        boost::program_options::command_line_parser(argc, argv)
            .options(opts_desc)
            // .positional(opts_pos)
            .run(),
        opts_vm);

    if (opts_vm.count("help"))
    {
      std::cout << opts_desc << std::endl;
      return 0;
    }
  }
  catch (boost::exception& e)
  {
    ROS_FATAL("[%s] something went wrong.... (error: %s)\n",
              ros::this_node::getName().c_str(),
              boost::diagnostic_information(e).c_str());
    std::cout << opts_desc << std::endl;
    return 0;
  }
  catch (...)
  {
    ROS_FATAL("[%s] unexpected behavior.....\n",
              ros::this_node::getName().c_str());
    std::cout << opts_desc << std::endl;
    return -1;
  }

  try
  {
    boost::program_options::notify(opts_vm);
  }
  catch (boost::exception& e)
  {
    ROS_FATAL("[%s] something went wrong.... (error: %s)\n",
              ros::this_node::getName().c_str(),
              boost::diagnostic_information(e).c_str());
    std::cout << opts_desc << std::endl;
    return -1;
  }
  catch (...)
  {
    ROS_FATAL("[%s] unexpected behavior.....\n",
              ros::this_node::getName().c_str());
    std::cout << opts_desc << std::endl;
    return -1;
  }

  if (opts_vm.count("duration"))
  {
    param.has_duration = true;
  }
  else
  {
    param.has_duration = false;
  }
  if (opts_vm.count("log"))
  {
    param.enable_log = true;
  }
  else
  {
    param.enable_log = false;
  }

  try
  {
    HectorMappingROS1Offline sm(param);

    sm.run();

    return 0;
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("[%s] Fatal error: %s", ros::this_node::getName().c_str(),
              e.what());

    ros::spinOnce();
    ros::requestShutdown();

    return -1;
  }
}
