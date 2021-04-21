

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <thread>

#include <geometry_msgs/Quaternion.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "map_builder/MapBuilder.h"
#include "utils/TicToc.h"

using namespace lio;
using namespace std;
using namespace mathutils;

static ros::NodeHandlePtr nh_ptr;

int main(int argc, char **argv) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  ros::init(argc, argv, "map_builder");

  ros::NodeHandle nh("~");

  MapBuilderConfig config;
  config.map_filter_size = 0.2;

  MapBuilder mapper(config);
  mapper.SetupRos(nh);
  mapper.Reset();

  ros::Rate r(100);
  while (ros::ok()) {
    mapper.ProcessMap();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
