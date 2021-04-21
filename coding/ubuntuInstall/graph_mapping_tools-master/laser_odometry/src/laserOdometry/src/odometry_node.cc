

#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Quaternion.h>

#include "point_processor/PointOdometry.h"
#include "utils/TicToc.h"

using namespace lio;
using namespace std;
using namespace mathutils;

DEFINE_int32(io_ratio, 2, "ratio of io");

int main(int argc, char **argv) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  ros::init(argc, argv, "point_odometry");

  ros::NodeHandle nh("~");

  PointOdometry odometry(0.1, FLAGS_io_ratio);
  odometry.SetupRos(nh);
  odometry.Reset();

  ros::Rate r(100);
  while (ros::ok()) {
    odometry.Process();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}