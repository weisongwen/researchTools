#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>



#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <graph_mapping_tools/ros_utils.hpp>
#include <graph_mapping_tools/registrations.hpp>

#include <graph_mapping_tools/ros_utils.hpp>


namespace graph_mapping_tools {

class ScanMatchingOdometryNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanMatchingOdometryNodelet() {}
  virtual ~ScanMatchingOdometryNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing scan_matching_odometry_nodelet...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe("/filtered_points", 256, &ScanMatchingOdometryNodelet::cloud_callback, this);
    imu_sub = nh.subscribe("/imu/data", 1024, &ScanMatchingOdometryNodelet::imu_callback, this);
    lego_laser_odom_prediction = nh.subscribe("/laser_odom_to_init1111",1024,&ScanMatchingOdometryNodelet::legoOdomPrediction, this);
    read_until_pub = nh.advertise<std_msgs::Header>("/scan_matching_odometry/read_until", 32);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 32);
    delta_yaw_pub = nh.advertise<geometry_msgs::Point32>("/delta_yaw", 32); // delta yaw of imu and ndt_odom
  }

  int frame_cnt =0;

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    auto& pnh = private_nh;
    points_topic = pnh.param<std::string>("points_topic", "/velodyne_points");
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 1.0);

    // Registration validation by thresholding
    transform_thresholding = pnh.param<bool>("transform_thresholding", false);
    max_acceptable_trans = pnh.param<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle = pnh.param<double>("max_acceptable_angle", 1.0);

    //use use_imu_ori_predi
    use_imu_ori_predi = pnh.param<double>("use_imu_ori_predi", 1.0);

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    std::string downsample_method = pnh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = pnh.param<double>("downsample_resolution", 0.1);
    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" <<std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      boost::shared_ptr<pcl::PassThrough<PointT>> passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }

    registration = select_registration_method(pnh);

    pre_orien.x =0.2; pre_orien.y =0.3; pre_orien.z =0.12; pre_orien.w =0.23; 
    cur_orien.x =0.2; cur_orien.y =0.3; cur_orien.z =0.12; cur_orien.w =0.23; 
  }

  void imu_callback(const sensor_msgs::ImuPtr& imu_msg) {
      
      tf::Quaternion q(
      imu_msg->orientation.x,
      imu_msg->orientation.y,
      imu_msg->orientation.z,
      imu_msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw, yaw_ori;
    m.getRPY(roll, pitch, yaw);

    roll = roll * 180.0 / 3.14159;
    pitch = pitch * 180.0 / 3.14159;
    yaw = yaw * 180.0 / 3.14159;
    cur_imu_yaw = yaw;
    cur_orien = imu_msg->orientation;
    // std::cout<<"cur_imu_yaw - pre_imu_yaw -> "<< cur_imu_yaw - pre_imu_yaw << std::endl;

  }

  void legoOdomPrediction(const nav_msgs::OdometryPtr& lego_odom_prediction_msg) {    
  // std::cout<<"cur_imu_yaw - pre_imu_yaw -> "<< cur_imu_yaw - pre_imu_yaw << std::endl;
  // nav_msgs::Odometry lego_odom_msg = *lego_odom_prediction_msg;
  Eigen::Isometry3d lego_odom = odom2isometry(*lego_odom_prediction_msg); // from ros odometry msg to Eigen::Isometry3d
  predict_trans = lego_odom * pre_lego_odom.inverse();
  // prev_trans.inverse() * trans

  
  predict_trans_mat = predict_trans.matrix().cast<float>(); // from Isometry3d to Matrix4f: get the predicted trans
  
  pre_lego_odom = lego_odom;

  }



  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    
    if(!ros::ok()) {
      return;
    }

    // std::cout<<"poitn cloud callback in scan matching odometry"<< std::endl;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    ros::Time before = ros::Time::now(); // start matching
    Eigen::Matrix4f pose = matching(cloud_msg->header.stamp, cloud);
    // std::cout<< "seconds used for matching-> "<<(ros::Time::now() - before).toSec() * 1000<<std::endl;
    frame_cnt++;
    // if(frame_cnt==2)
    {
      publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);
      frame_cnt = 0;
    }
    

    // In offline estimation, point clouds until the published time will be supplied
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);


  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  /**
   * @brief estimate the relative pose between an input cloud and a keyframe cloud
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the keyframe cloud
   */
  Eigen::Matrix4f matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe) {
      prev_trans.setIdentity();
      keyframe_pose.setIdentity();
      keyframe_stamp = stamp;
      keyframe = downsample(cloud);
      registration->setInputTarget(keyframe);
      return Eigen::Matrix4f::Identity();
    }
    ++cloud_cnt;
    auto filtered = downsample(cloud);
    registration->setInputSource(filtered);

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    // prev_trans.setIdentity();

    if(use_imu_ori_predi) // use imu orientation for trans nintial guess
    {
      Eigen::Quaternionf pre_trans_quat(prev_trans.block<3, 3>(0, 0));
      tf::Quaternion tf_q(pre_trans_quat.x(), pre_trans_quat.y(), pre_trans_quat.z(), pre_trans_quat.w());
      tf::Matrix3x3 m33(tf_q);
      double ndt_delta_roll=0, ndt_delta_pitch=0, ndt_delta_yaw=0;
      m33.getRPY(ndt_delta_roll, ndt_delta_pitch, ndt_delta_yaw);
      ndt_delta_yaw = ndt_delta_yaw * 180.0 / 3.14159;
      
      delta_imu_yaw = cur_imu_yaw - pre_imu_yaw;
      pre_imu_yaw = cur_imu_yaw;
      geometry_msgs::Point32 delta_yaw_p32;
      delta_yaw_p32.x = ndt_delta_yaw;
      delta_yaw_p32.y = delta_imu_yaw;
      delta_yaw_p32.z = 0;
      delta_yaw_pub.publish(delta_yaw_p32);
      // std::cout<<"ndt_delta_yaw-> "<< ndt_delta_yaw << "     delta_imu_yaw-> " <<delta_imu_yaw <<std::endl;
      
      if(pre_orien.x = NULL)
      {
        pre_orien = cur_orien;
      }

      Eigen::Matrix3d pre_R = Eigen::Matrix3d::Identity();
      // Eigen::Quaternion Q1;
      tf::Quaternion pre_q(
        pre_orien.x,
        pre_orien.y,
        pre_orien.z,
        pre_orien.w);
      tf::Matrix3x3 pre_m(pre_q);
      // std::cout<<"pre_m(0,0)-> " << pre_m[0][1]<<std::endl;
      pre_R(0,0) = pre_m[0][0]; pre_R(0,1) = pre_m[0][1]; pre_R(0,2) = pre_m[0][2];
      pre_R(1,0) = pre_m[1][0]; pre_R(1,1) = pre_m[1][1]; pre_R(1,2) = pre_m[1][2];
      pre_R(2,0) = pre_m[2][0]; pre_R(2,1) = pre_m[2][1]; pre_R(2,2) = pre_m[2][2];

      Eigen::Matrix3d cur_R = Eigen::Matrix3d::Identity();
      // Eigen::Quaternion Q1;
      tf::Quaternion cur_q(
        cur_orien.x,
        cur_orien.y,
        cur_orien.z,
        cur_orien.w);
      tf::Matrix3x3 cur_m(cur_q);
      // std::cout<<"cur_m(0,0)-> " << cur_m[0][1] <<std::endl;
      cur_R(0,0) = cur_m[0][0]; cur_R(0,1) = cur_m[0][1]; cur_R(0,2) = cur_m[0][2];
      cur_R(1,0) = cur_m[1][0]; cur_R(1,1) = cur_m[1][1]; cur_R(1,2) = cur_m[1][2];
      cur_R(2,0) = cur_m[2][0]; cur_R(2,1) = cur_m[2][1]; cur_R(2,2) = cur_m[2][2];

      // Q1.x()
      Eigen::Matrix3d delta_R = pre_R.inverse() * cur_R;
      // Eigen::Matrix3d delta_R = cur_R.inverse() * pre_R;

      if(delta_imu_yaw<500 && (cloud_cnt>30)) // 
      {
        cloud_cnt =30+10;
        prev_trans << delta_R(0,0), delta_R(0,1), delta_R(0,2), prev_trans(0,3),
                    delta_R(1,0), delta_R(1,1), delta_R(1,2), prev_trans(1,3),
                    delta_R(2,0), delta_R(2,1), delta_R(2,2), prev_trans(2,3),
                    prev_trans(3,0), prev_trans(3,1), prev_trans(3,2), prev_trans(3,3);
        double dx = prev_trans.block<3, 1>(0, 3).norm();
        double da = std::acos(Eigen::Quaternionf(prev_trans.block<3, 3>(0, 0)).w());
        std::cout<<"dx-> " << dx  << "  da-> " << da<<std::endl;
      }
      pre_orien = cur_orien;
    }
    
    
    // registration->align(*aligned, predict_trans_mat);
    registration->align(*aligned, prev_trans);

    if(!registration->hasConverged()) {
      NODELET_INFO_STREAM("scan matching has not converged!!");
      NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
      return keyframe_pose * prev_trans;
    }

    Eigen::Matrix4f trans = registration->getFinalTransformation();
    Eigen::Matrix4f odom = keyframe_pose * trans;

    if(transform_thresholding) {
      Eigen::Matrix4f delta = prev_trans.inverse() * trans;
      double dx = delta.block<3, 1>(0, 3).norm();
      double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());

      if(dx > max_acceptable_trans || da > max_acceptable_angle) {
        NODELET_INFO_STREAM("too large transform!!  " << dx << "[m] " << da << "[rad]");
        NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
        return keyframe_pose * prev_trans;
      }
    }

    prev_trans = trans;

    auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    // keyframe_broadcaster.sendTransform(keyframe_trans);

    double delta_trans = trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double delta_time = (stamp - keyframe_stamp).toSec();
    if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
      keyframe = filtered;
      registration->setInputTarget(keyframe);

      keyframe_pose = odom;
      keyframe_stamp = stamp;
      // std::cout << "the prediction transfrom is set to zero" << std::endl;
      prev_trans.setIdentity();
    }

    return odom;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
    // odom_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;

    // odom.pose.pose.position.x = pose(0, 3);
    // odom.pose.pose.position.y = pose(1, 3);
    // odom.pose.pose.position.z = pose(2, 3);
    // odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.pose.pose.position.x = pose(1, 3);
    odom.pose.pose.position.y = pose(2, 3);
    odom.pose.pose.position.z = pose(0, 3);
    odom.pose.pose.orientation.x = odom_trans.transform.rotation.y;
    odom.pose.pose.orientation.y = odom_trans.transform.rotation.z;
    odom.pose.pose.orientation.z = odom_trans.transform.rotation.x;
    odom.pose.pose.orientation.w = odom_trans.transform.rotation.w;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom.header.frame_id = "/camera_init";
    odom.child_frame_id = "/laser_odom";

    odom_pub.publish(odom);
  }


private:
  // ROS topics
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber lego_laser_odom_prediction;

  ros::Publisher odom_pub;
  ros::Publisher delta_yaw_pub;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster keyframe_broadcaster;


  std::string points_topic;
  std::string odom_frame_id;
  ros::Publisher read_until_pub;

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  // odometry calculation
  Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;               // keyframe pose
  ros::Time keyframe_stamp;                    // keyframe time
  pcl::PointCloud<PointT>::ConstPtr keyframe;  // keyframe point cloud

  //
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // IMU
  double pre_imu_yaw, cur_imu_yaw, delta_imu_yaw;
  geometry_msgs::Quaternion pre_orien, cur_orien;
  double use_imu_ori_predi;

  // cloud count
  double cloud_cnt=0;

  // lego loam 
  Eigen::Isometry3d pre_lego_odom;
  Eigen::Isometry3d predict_trans;
  Eigen::Matrix4f predict_trans_mat; // pose in Eigen::Matrix4f format
};

}

PLUGINLIB_EXPORT_CLASS(graph_mapping_tools::ScanMatchingOdometryNodelet, nodelet::Nodelet)
