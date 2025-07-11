# [hector_slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam)

See the ROS Wiki for documentation: http://wiki.ros.org/hector_slam

**Modified by Ricardo B. Sousa**

This fork only adds an offline node to process multiple ROS bag files, generate
the 2D occupancy grid map through ROS API, and save the robot pose data into TUM
format (need to check if the pose data is indeed laser pose, or for the base).

## Usage

```sh
$ rosrun hector_mapping hector_mapping_offline --help
Usage: rosrun hector_mapping hector_slam_offline -b BAGFILE1 [BAGFILE2 BAGFILE3 ...]

Options:
  -h [ --help ]                         Display this information.
  -b [ --bags ] BAGFILE                 ROS bag files to process
  --initialposetopic TOPIC (=/initial_pose)
                                        Topic name for the robot's initial pose
  --scantopic TOPIC (=/scan)            Topic name for the 2D laser scanner
                                        data
  -s [ --start ] SEC (=0)               start SEC seconds into the bag files
  -d [ --duration ] SEC                 play only SEC seconds from the bag
                                        files
  --log FILENAME                        log the robot estimated data (laser
                                        pose) into TUM files
```

See the ROS launch file
[mapping_offline.launch](/hector_mapping/launch/mapping_offline.launch)
for an example on how to launch the node similar to the online version
(including parametrizing the HectorSLAM system through ROS parameters).
