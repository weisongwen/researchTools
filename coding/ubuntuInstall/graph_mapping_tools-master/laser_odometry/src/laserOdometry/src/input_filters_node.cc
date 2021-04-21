

#include <utils/common_ros.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>

using namespace lio;

static ros::Publisher pub_filtered_cloud;
static Eigen::Affine3f transform_to_world;
static tf::TransformBroadcaster *br_ptr;
static tf::Transform tf_transform;

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &raw_points_msg) {

  // fetch new input cloud
  pcl::PointCloud<pcl::PointXYZI> laser_cloud_in, tmp_cloud;
  pcl::fromROSMsg(*raw_points_msg, laser_cloud_in);
//  pcl::PointCloud<pcl::PointXYZI>::ConstPtr laser_cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>(laser_cloud_in));
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI> laser_cloud_out;

  pcl::transformPointCloud(laser_cloud_in, *transformed_ptr, transform_to_world);

  pcl::CropBox<pcl::PointXYZI> box_filter;
  box_filter.setMin(Eigen::Vector4f(-10, -5, -1.7, 1.0));
  box_filter.setMax(Eigen::Vector4f(5, 7, 0.6, 1.0));
  box_filter.setNegative(true);
  box_filter.setInputCloud(transformed_ptr);
  box_filter.filter(tmp_cloud);

  pcl::transformPointCloud(tmp_cloud, laser_cloud_out, transform_to_world.inverse());

  PublishCloudMsg(pub_filtered_cloud,
                  laser_cloud_out,
                  raw_points_msg->header.stamp,
                  raw_points_msg->header.frame_id);

  br_ptr->sendTransform(tf::StampedTransform(tf_transform, raw_points_msg->header.stamp, "imu_link_kaist", "right_velodyne"));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "input_filters");
  ros::NodeHandle nh("~");

  ros::Subscriber sub_raw_points = nh.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_points", 2, PointCloudHandler);

  br_ptr = new tf::TransformBroadcaster();
  pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 2);

  Eigen::Matrix3f R_inv;
  R_inv << -4.91913910e-01, -5.01145813e-01, -7.11950546e-01,
      7.13989130e-01, -7.00156621e-01, -4.78439170e-04,
      -4.98237120e-01, -5.08560301e-01, 7.02229444e-01;
  transform_to_world.setIdentity();
  transform_to_world.linear() = R_inv.transpose();


  Eigen::Quaternionf q_eigen(R_inv.transpose());
  tf::Quaternion q(tf::Quaternion{q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w()});
  tf_transform.setRotation(q);

  ros::Rate r(500);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}