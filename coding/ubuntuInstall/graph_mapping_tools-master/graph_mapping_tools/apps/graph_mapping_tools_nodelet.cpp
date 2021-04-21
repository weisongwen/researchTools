#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <graph_mapping_tools/FloorCoeffs.h>

#include <graph_mapping_tools/SaveMap.h>
#include <graph_mapping_tools/DumpGraph.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <graph_mapping_tools/ros_utils.hpp>
#include <graph_mapping_tools/ros_time_hash.hpp>

#include <graph_mapping_tools/graph_slam.hpp>
#include <graph_mapping_tools/keyframe.hpp>
#include <graph_mapping_tools/keyframe_updater.hpp>
#include <graph_mapping_tools/loop_detector.hpp>
#include <graph_mapping_tools/information_matrix_calculator.hpp>
#include <graph_mapping_tools/map_cloud_generator.hpp>
#include <graph_mapping_tools/nmea_sentence_parser.hpp>

#include <g2o/types/slam3d/edge_se3.h> // constraints
#include <g2o/types/slam3d/vertex_se3.h> // nodes 
#include <g2o/edge_se3_plane.hpp> // plane constraints
#include <g2o/edge_se3_priorxy.hpp> // gps 2D (x and y) positional constraints
#include <g2o/edge_se3_priorxyz.hpp> // gps 3D positional (x,y and z)
#include <g2o/edge_se3_priorvec.hpp> // velocity constraint
#include <g2o/edge_se3_priorquat.hpp> // quaternion constaint

#include <novatel_msgs/BESTPOS.h>
#include <novatel_msgs/INSPVAX.h>
#include <graph_mapping_tools/gnss_tools.hpp>
#include <geometry_msgs/Point32.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/NavSatFix.h> // for publish the SLAM pose in llh


FILE* fp_out_gra_pose = fopen("/home/wws/Downloads/JianDong/data2meng/LiDAR.csv", "w+"); //


namespace graph_mapping_tools {

class HdlGraphSlamNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT; // rename

  HdlGraphSlamNodelet() {}
  virtual ~HdlGraphSlamNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // init parameters
    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
    map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
    trans_odom2map.setIdentity(); // optimized compensate

    max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);

    //
    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    nmea_parser.reset(new NmeaSentenceParser());

    gps_time_offset = private_nh.param<double>("gps_time_offset", 0.0); // not used
    gps_edge_stddev_xy = private_nh.param<double>("gps_edge_stddev_xy", 10000.0); // gps STD for GNSS XY
    gps_edge_stddev_z = private_nh.param<double>("gps_edge_stddev_z", 10.0); // get STD for GNSS z
    floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0); // get STD for floor edge

    imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0); // 
    enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false); // use imu orientation
    enable_imu_acceleration = private_nh.param<bool>("enable_imu_acceleration", false); // use imu acceleration
    imu_orientation_edge_stddev = private_nh.param<double>("imu_orientation_edge_stddev", 0.1); // get STD for orientation edge
    imu_acceleration_edge_stddev = private_nh.param<double>("imu_acceleration_edge_stddev", 3.0); // get STD for acc edge

    enable_loop_detection = private_nh.param<bool>("enable_loop_detection", false); // get the bool for loop detection 

    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_poits"); // get point clouds topic

    // subscribers aft_mapped_to_init_10
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/aft_mapped_to_init_10", 256)); // subscriber from loam
    // odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/integrated_to_init", 256)); // subscriber from lego-loam
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 32)); // subscriber for filtered points 
    sync.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>(*odom_sub, *cloud_sub, 32)); // allign the two subscribers
    sync->registerCallback(boost::bind(&HdlGraphSlamNodelet::cloud_callback, this, _1, _2)); // allign the two subscribers
    imu_sub = nh.subscribe("/imu/data", 1024, &HdlGraphSlamNodelet::imu_callback, this); // subscribe IMU message
    floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024, &HdlGraphSlamNodelet::floor_coeffs_callback, this); // subscribe floor coeffs

    if(private_nh.param<bool>("enable_gps", true)) {
      gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &HdlGraphSlamNodelet::gps_callback, this); // subscribe gps message
      nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &HdlGraphSlamNodelet::nmea_callback, this); // subscribe gps message
      navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &HdlGraphSlamNodelet::navsat_callback, this); // subscribe gps message
      spancpt_sub = nh.subscribe("/novatel_data/inspvax",1024, &HdlGraphSlamNodelet::inspvax_callback, this); // subscribe gps message from span-cpt
      spancpt_markers_pub = mt_nh.advertise<visualization_msgs::Marker>("/graph_mapping_tools/spancpt_marker",16); // publish span-cpt markers
    }
    
    // subscribe the positioning from loam: not useful if it is directly applied
    sub_laser_odometry_ = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_10_not_used", 2, &HdlGraphSlamNodelet::LaserOdometryHandler, this);
    // publishers
    markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/graph_mapping_tools/markers", 16); // graph SLAM related markers
    odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/graph_mapping_tools/odom2pub", 16); // transform from odom to map 
    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/graph_mapping_tools/map_points", 1); // map points
    read_until_pub = mt_nh.advertise<std_msgs::Header>("/graph_mapping_tools/read_until", 32); // 
    graph_pose_pub = mt_nh.advertise<nav_msgs::Odometry>("/graph_mapping_tools/graph_pose", 32); // optimized pose of base_link
    yaw_pub = mt_nh.advertise<geometry_msgs::Point32>("/yaw", 32); // yaw angle from span-cpt graph_pose and imu (AHRS)

    mapped_pose_pub = mt_nh.advertise<nav_msgs::Odometry>("/graph_mapping_tools/mapped_pose", 32); // mapped pose of base_link


    dump_service_server = mt_nh.advertiseService("/graph_mapping_tools/dump", &HdlGraphSlamNodelet::dump_service, this); // save graph SLAM related results
    save_map_service_server = mt_nh.advertiseService("/graph_mapping_tools/save_map", &HdlGraphSlamNodelet::save_map_service, this); // save point cloud maps

    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0); // get interval/frequency for optimization
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0); // get interval/frequency for map updates
    optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &HdlGraphSlamNodelet::optimization_timer_callback, this); // timer
    map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &HdlGraphSlamNodelet::map_points_publish_timer_callback, this); // timer
    
    pub_car = nh.advertise<visualization_msgs::MarkerArray>("car_model", 1000);

    pub_odom_llh = nh.advertise<sensor_msgs::NavSatFix>("/odom2navsat", 1000);

    first_imu_msg.orientation.w = 0; // get first msg from orientaion
    first_imu_msg.orientation.x = 0; 
    first_imu_msg.orientation.y = 0;
    first_imu_msg.orientation.z = 0;

    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(1.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    registration = ndt;

  }

private:
  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg 
   */
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) { // subscribe point cloud and odom
    const ros::Time& stamp = odom_msg->header.stamp; // ros time 
    Eigen::Isometry3d odom = odom2isometry(*odom_msg); // from ros odometry msg to Eigen::Isometry3d
    Eigen::Isometry3d iso_mapped_odom ; // from ros odometry msg to Eigen::Isometry3d

    Eigen::Isometry3d pose_est = map2odomTrans *  odom; // odometry to first frame
    Eigen::Matrix4f pose_est_mat; // pose in Eigen::Matrix4f format
    pose_est_mat = pose_est.matrix().cast<float>(); // from Isometry3d to Matrix4f

    // publish the tf 
    geometry_msgs::TransformStamped odom2velodyne_trans = matrix2transform(stamp, odom.matrix().cast<float>(), "/odom_graph_tools", "/base_link_graph_tools");
    odom2velodyne_broadcaster.sendTransform(odom2velodyne_trans);

    Eigen::Quaternionf quat(pose_est_mat.block<3, 3>(0, 0));
    quat.normalize();
    // publish pose of the final optimization
    nav_msgs::Odometry odom_;
    odom_.header.stamp = odom_msg->header.stamp; //stamp
    odom_.header.frame_id = "odom_graph_tools";
    odom_.pose.pose.position.x = pose_est(0, 3);
    odom_.pose.pose.position.y = pose_est(1, 3);
    odom_.pose.pose.position.z = pose_est(2, 3);

    odom_.pose.pose.orientation.w = quat.w();
    odom_.pose.pose.orientation.x = quat.x();
    odom_.pose.pose.orientation.y = quat.y();
    odom_.pose.pose.orientation.z = quat.z();

    odom_.child_frame_id = "velodyne";
    odom_.twist.twist.linear.x = 0.0;
    odom_.twist.twist.linear.y = 0.0;
    odom_.twist.twist.angular.z = 0.0;
    graph_pose_pub.publish(odom_);

    bool publish_car_model = 1;
    if(publish_car_model) // publish the car model from VINS
    {
      visualization_msgs::MarkerArray markerArray_msg;
      visualization_msgs::Marker car_mesh;
      car_mesh.header.stamp = odom_msg->header.stamp;
      car_mesh.header.frame_id = "/map_graph_tools";
      car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
      car_mesh.action = visualization_msgs::Marker::ADD;
      car_mesh.id = 0;

      car_mesh.mesh_resource = "package://graph_mapping_tools/models/car.dae";

      Eigen::Matrix3d rot;
      double angle_y = 3.1415926 / 2;
      Eigen::AngleAxisd rotation_vector(3.1415926 / 2,Eigen::Vector3d(1,0,0)); // rotate a certain angle
      rot = rotation_vector.matrix();

      // rot << 0, 0, -1, 
      //        0, -1, 0, 
      //        -1, 0, 0;
      
      Eigen::Quaterniond Q;
      Eigen::Quaterniond quat_t;
      const auto& orientation = odom_.pose.pose.orientation;
      quat_t.w() = orientation.w;
      quat_t.x() = orientation.x;
      quat_t.y() = orientation.y;
      quat_t.z() = orientation.z;
      // quat_t.normalize();
      Q = quat_t * rot;    
      car_mesh.pose.position.x    = pose_est(0, 3);
      car_mesh.pose.position.y    = pose_est(1, 3);
      car_mesh.pose.position.z    = pose_est(2, 3) -2 ;
      car_mesh.pose.orientation.w = Q.w();
      car_mesh.pose.orientation.x = Q.x();
      car_mesh.pose.orientation.y = Q.y();
      car_mesh.pose.orientation.z = Q.z();

      car_mesh.color.a = 1.0;
      car_mesh.color.r = 1.0;
      car_mesh.color.g = 0.0;
      car_mesh.color.b = 0.0;

      float major_scale = 1.0; // scale for the car model

      car_mesh.scale.x = major_scale;
      car_mesh.scale.y = major_scale;
      car_mesh.scale.z = major_scale;
      markerArray_msg.markers.push_back(car_mesh);
      pub_car.publish(markerArray_msg);
    }

    bool save_trajectory = 1; // data in jiandong, 0428, prepare for Dr. Meng, this is useful for the data before, 20190428
    if(save_trajectory && origin_set)
    {
      if (fp_out_gra_pose == NULL) {
      std::cout << "/* Error opening file!! */" << std::endl;
      }

      double prex_ = pose_est(0, 3);
      double prey_ = pose_est(1, 3);
      double prez_ = pose_est(2, 3);

      double origin_azimuth_ = origin_azimuth + 180 - 0 - 180; // data in matoujiao 
      double theta = -1 * (origin_azimuth_ - 90)*( 3.141592 / 180.0 );
      pose_est(0, 3) = (prex_ * cos(theta) - prey_ * sin(theta));
      pose_est(1, 3) = (prex_ * sin(theta) + prey_ * cos(theta));
      
      bool publish2llh = 1; // generate the LiDAR-based ground truth for PNT 2019
      if(publish2llh) // publsih the ENU 
      {
        Eigen::MatrixXd LiDAR2ENU;
        LiDAR2ENU.resize(3, 1);
        LiDAR2ENU(0) = pose_est(0, 3);
        LiDAR2ENU(1) = pose_est(1, 3);
        LiDAR2ENU(2) = pose_est(2, 3);
        Eigen::MatrixXd ECEF_ = gnss_tools_.enu2ecef(originllh, LiDAR2ENU);
        Eigen::MatrixXd lidarOdom2llh = gnss_tools_.ecef2llh(ECEF_);
        sensor_msgs::NavSatFix odom2navfix;
        odom2navfix.header = odom_.header;
        odom2navfix.longitude = lidarOdom2llh(0);
        odom2navfix.latitude = lidarOdom2llh(1);
        odom2navfix.altitude = lidarOdom2llh(2);
        pub_odom_llh.publish(odom2navfix);
      }

      fprintf(fp_out_gra_pose, "%3.7f,%3.7f,%3.7f,%3.7f\n", stamp.toSec(), pose_est(0, 3),pose_est(1, 3),pose_est(2, 3));
    }
    

    bool use_mapping = 0;
    if(use_mapping && globalmap_updated)
    {
      const clock_t begin_time = clock();

      registration->setInputTarget(globalmap);
      pcl::PointCloud<PointT>::Ptr cloud_for_mapping(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*cloud_msg, *cloud_for_mapping);
      pcl::PointCloud<PointT>::Ptr alignedfor_mapping(new pcl::PointCloud<PointT>());
      registration->setInputSource(cloud_for_mapping);

      Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
      Eigen::Isometry3d Iso_odom_ = odom2isometry(odom_); // from ros odometry msg to Eigen::Isometry3d
      init_guess = odom.matrix().cast<float>(); // from Isometry3d to Matrix4f
      registration->align(*alignedfor_mapping, init_guess);
      Eigen::Matrix4f mapped_trans = registration->getFinalTransformation();
      std::cout << "mapping used time -> " << double(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";
      // publish pose of the mapped 
      Eigen::Vector3f mapped_p = mapped_trans.block<3, 1>(0, 3);
      Eigen::Quaternionf mapped_q(mapped_trans.block<3, 3>(0, 0));
      mapped_q.normalize();
      nav_msgs::Odometry odom_mapped;
      odom_mapped.header.stamp = odom_msg->header.stamp; //stamp
      odom_mapped.header.frame_id = "odom_graph_tools";
      odom_mapped.pose.pose.position.x = mapped_p(0);
      odom_mapped.pose.pose.position.y = mapped_p(1);
      odom_mapped.pose.pose.position.z = mapped_p(2);

      odom_mapped.pose.pose.orientation.w = mapped_q.w();
      odom_mapped.pose.pose.orientation.x = mapped_q.x();
      odom_mapped.pose.pose.orientation.y = mapped_q.y();
      odom_mapped.pose.pose.orientation.z = mapped_q.z();

      odom_mapped.child_frame_id = "velodyne";
      odom_mapped.twist.twist.linear.x = 0.0;
      odom_mapped.twist.twist.linear.y = 0.0;
      odom_mapped.twist.twist.angular.z = 0.0;
      // mapped_pose_pub.publish(odom_mapped);

      Eigen::Isometry3d tmp_odom = odom2isometry(odom_mapped);
      tmp_odom = tmp_odom * map2odomTrans.inverse();

      Eigen::Matrix4f pose_mat; // pose in Eigen::Matrix4f format
      pose_mat = tmp_odom.matrix().cast<float>(); // from Isometry3d to Matrix4f

      Eigen::Quaternionf quat_tmp(pose_mat.block<3, 3>(0, 0));
      quat_tmp.normalize();
      // publish pose of the final optimization
      odom_mapped.pose.pose.position.x = pose_mat(0, 3);
      odom_mapped.pose.pose.position.y = pose_mat(1, 3);
      odom_mapped.pose.pose.position.z = pose_mat(2, 3);

      odom_mapped.pose.pose.orientation.w = quat_tmp.w();
      odom_mapped.pose.pose.orientation.x = quat_tmp.x();
      odom_mapped.pose.pose.orientation.y = quat_tmp.y();
      odom_mapped.pose.pose.orientation.z = quat_tmp.z();
      mapped_pose_pub.publish(odom_mapped);
      iso_mapped_odom = odom2isometry(odom_mapped);
    }

    tf::Quaternion q(
      quat.x(),
      quat.y(),
      quat.z(),
      quat.w());
    tf::Matrix3x3 m(q); // tf::Matrix3x3 
    double roll, pitch, yaw, yaw_ori; // 
    m.getRPY(roll, pitch, yaw); // get roll pitch yaw from matrix 3X3
    yaw_ori = yaw;
    yaw = yaw * 180.0 / 3.14159;
    graph_pose_yaw = yaw;
    if(yaw > 180) yaw = yaw -360.0;
    if(yaw < -180) yaw = yaw +360.0;
    // std::cout<< "yaw angle" << yaw << std::endl;
    geometry_msgs::Point32 yaw_point32;
    yaw_point32.x = span_yaw;
    // std::cout<<"span-cpt yaw " << span_yaw <<std::endl;
    yaw_point32.y = graph_pose_yaw;
    yaw_point32.z = imu_yaw;
    yaw_pub.publish(yaw_point32);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(base_frame_id.empty()) {
      base_frame_id = cloud_msg->header.frame_id;
    }

    if(!keyframe_updater->update(odom)) { // update the odom to the keyframe_updater
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        std_msgs::Header read_until;
        read_until.stamp = stamp + ros::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }

      return;
    }

    double accum_d = keyframe_updater->get_accum_distance(); // get the accumulated distance
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud)); // update key frame

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe); // save keyframe to queen
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) {
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    int num_processed = 0;
    for(int i=0; i<std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      // new_keyframes will be tested later for loop closure
      new_keyframes.push_back(keyframe);

      // add pose node
      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam->add_se3_node(odom);
      keyframe_hash[keyframe->stamp] = keyframe;

      // fix the first node
      if(keyframes.empty() && new_keyframes.size() == 1) {
        if(private_nh.param<bool>("fix_first_node", false)) {
          anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
          anchor_node->setFixed(true);
          anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 6));
        }
      }

      if(i==0 && keyframes.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom; // relative pose between two NDT odom
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(prev_keyframe->cloud, keyframe->cloud, relative_pose); // calculate information matrix
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information); // add the keyframe 
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
    }

    std_msgs::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(10, 0);
    read_until.frame_id = points_topic;
    read_until_pub.publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1); // clear the keyframe queen 
    return true;
  }

  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) { // subscribe the msgs from nmea
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

    if(grmc.status != 'A') {
      return;
    }

    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped()); // subscribe the msgs from geographic_msgs
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
  }

  void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) { // subscribe the msgs from navsat
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;
    gps_callback(gps_msg);
  }

  void LaserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laser_odom_msg) {
  // std::cout<<"<-----test------>"<<std::endl;
  nav_msgs::Odometry odom_tmp = *laser_odom_msg;
  geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
  gps_msg->header = laser_odom_msg->header;
  gps_msg->position.longitude = odom_tmp.pose.pose.position.x;
  gps_msg->position.latitude = odom_tmp.pose.pose.position.y;
  gps_msg->position.altitude = odom_tmp.pose.pose.position.z;
  gps_callback(gps_msg);

  }


  void inspvax_callback(const novatel_msgs::INSPVAXPtr& novatel_msg){ //subscribe span-cpt solution
    double yaw = novatel_msg->azimuth;
    // if(yaw > 180) yaw = yaw -360.0;
    // if(yaw < -180) yaw = yaw +360.0;
    span_yaw = yaw;
    // std::cout<<"novatel_msg->azimuth " << novatel_msg->azimuth <<std::endl;
    Eigen::MatrixXd curLLh; // 
    Eigen::MatrixXd eigenENU; // 
    Eigen::MatrixXd ecef_origin;
    ecef_origin.resize(3, 1);
    double prex_, prey_, theta;
    eigenENU.resize(3, 1);
    curLLh.resize(3, 1);

    curLLh(0) = novatel_msg->longitude;
    curLLh(1) = novatel_msg->latitude;
    curLLh(2) = novatel_msg->altitude;

    Eigen::MatrixXd ecef;
    ecef.resize(3, 1);
    ecef = gnss_tools_.llh2ecef(curLLh);
    cptspan_count++;

    if (cptspan_count == 1) { // get first epoch
        origin_set = true;
        originllh = curLLh;
        std::cout << std::setprecision(17);
        originllh.resize(3, 1);
        origin_azimuth = novatel_msg->azimuth;
      }

    if (origin_set) {
      std::cout << std::setprecision(17);
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh,ecef);

      // std::cout << "xENU_ori = " << eigenENU(0) << " yENU_ori = " << eigenENU(1) << " zENU_ori = " << eigenENU(2) << std::endl;
      prex_ = eigenENU(0);
      prey_ = eigenENU(1);
      theta = (origin_azimuth+90.0)*( 3.141592 / 180.0 ); //
      eigenENU(0) = (prex_ * cos(theta) - prey_ * sin(theta));
      eigenENU(1) = (prex_ * sin(theta) + prey_ * cos(theta));
      // std::cout << "xENU = " << eigenENU(0) << " yENU = " << eigenENU(1) << std::endl;
      
      /** positioning type for GNSS/INS (SPAN-CPT)
      19 - PROPOGATED (Propagated by a Kalman filter without new observations)

      34 - NARROW_FLOAT (Floating narrow-lane ambiguity solution)

      53 - INS_PSRSP (INS pseudorange single point solution – no DGPS corrections)

      54 - INS_PSRDIFF (INS pseudorange differential solution)

      55 - INS_RTKFLOAT (INS RTK floating point ambiguities solution)

      56 - Other Types No 56 type (INS_RTKFIXED) ?
      **/

      std:: cout << "position_type = " <<  novatel_msg->position_type << std:: endl;
        //if (novatel_msg->position_type == 54 || novatel_msg->position_type == 55 || novatel_msg->position_type == 56) {
      spancpt_marker.header.frame_id = "map_graph_tools";
      spancpt_marker.header.stamp = ros::Time();
      spancpt_marker.ns = "spancpt";
      spancpt_marker.id = cptspan_count;
      spancpt_marker.type = visualization_msgs::Marker::SPHERE;

      spancpt_marker.pose.position.x = eigenENU(0);
      spancpt_marker.pose.position.y = eigenENU(1);
      spancpt_marker.pose.position.z = eigenENU(2);
      spancpt_marker.pose.orientation.w = 1.0;
      spancpt_marker.scale.x = spancpt_marker.scale.y = spancpt_marker.scale.z = 1.0;

      spancpt_marker.color.a = 1.0;
      spancpt_marker.color.r = 0.0;
      spancpt_marker.color.g = 1.0;
      spancpt_markers_pub.publish(spancpt_marker);

      geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
      gps_msg->header = imu_header;
      gps_msg->position.longitude = eigenENU(0);
      gps_msg->position.latitude = eigenENU(1);
      gps_msg->position.altitude = eigenENU(2);
      
      // gps_callback(gps_msg);
      // std::cout<<"push back message to gps_queue..."<<std::endl;
        //}
    }
    
}
  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::GeoPointStampedPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_msg->header.stamp += ros::Duration(gps_time_offset);
    if(gps_time_offset>0)
    {
      std::cout<<"<<--Warning-->> the gps_time_offset is larger than 0 -> "<< gps_time_offset <<std::endl;
    }
    gps_queue.push_back(gps_msg);
  }

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty() || gps_queue.empty()) {
      return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > gps_queue.back()->header.stamp) {
        break;
      }

      if(keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord) {
        continue;
      }

      // find the gps data which is closest to the keyframe
      auto closest_gps = gps_cursor;
      for(auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
        auto dt = ((*closest_gps)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*gps)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_gps = gps;
      }

      // if the time residual between the gps and keyframe is too large, skip it
      gps_cursor = closest_gps;
      if(0.21 < std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec())) {
        
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      geodesy::UTMPoint utm;
      geodesy::fromMsg((*closest_gps)->position, utm);
      // Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);
      Eigen::Vector3d xyz((*closest_gps)->position.longitude, (*closest_gps)->position.latitude, (*closest_gps)->position.altitude);

      // the first gps data position will be the origin of the map
      // if(!zero_utm) {
      //   zero_utm = xyz;
      // }
      // xyz -= (*zero_utm);

      keyframe->utm_coord = xyz;

      g2o::OptimizableGraph::Edge* edge;

      // 2D constraint
      // if(std::isnan(xyz.z())){ 
      if(1){ 
        Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
        edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
        std::cout<<"add gps constraints........................." << std::endl;
        std::cout<<"gps-LiDAR time residual ................................." <<std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec())<<  std::endl;
      } 
      // 3D constraint
      else {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
        information_matrix(2, 2) /= gps_edge_stddev_z;
        // edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
      }
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));

      updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp,
      [=](const ros::Time& stamp, const geographic_msgs::GeoPointStampedConstPtr& geopoint) {
        return stamp < geopoint->header.stamp;
      }
    );
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated; // gps queen successfully updated into the graph 
  }

  void imu_callback(const sensor_msgs::ImuPtr& imu_msg) {
      imu_header = imu_msg->header;
      if((first_imu_msg.orientation.w == NULL) && (enable_imu_orientation || enable_imu_acceleration))
      {
        first_imu_msg = *imu_msg;
        std::cout << "successfully initialize first imu msg.."<<std::endl;
      }
      tf::Quaternion q(
      imu_msg->orientation.x,
      imu_msg->orientation.y,
      imu_msg->orientation.z,
      imu_msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw, yaw_ori;
    m.getRPY(roll, pitch, yaw);
    yaw_ori = yaw;

    roll = roll * 180.0 / 3.14159;
    pitch = pitch * 180.0 / 3.14159;
    yaw = yaw * 180.0 / 3.14159;
    imu_cnt++;
    if(imu_cnt <100)
    {
      imu_first_yaw = yaw;
      imu_first_roll = roll;
      imu_first_pitch = pitch;
    }
    if(imu_cnt >10000)
        imu_cnt=0;

    if(!enable_imu_orientation && !enable_imu_acceleration) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    imu_msg->header.stamp += ros::Duration(imu_time_offset);
    imu_queue.push_back(imu_msg);
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if(keyframes.empty() || imu_queue.empty() || base_frame_id.empty()) {
      return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > imu_queue.back()->header.stamp) {
        break;
      }

      if(keyframe->stamp < (*imu_cursor)->header.stamp || keyframe->acceleration) {
        continue;
      }

      // find imu data which is closest to the keyframe
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
        auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if(0.2 < std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec())) {
        continue;
      }

          tf::Quaternion q(
      (*closest_imu)->orientation.x,
      (*closest_imu)->orientation.y,
      (*closest_imu)->orientation.z,
      (*closest_imu)->orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw, yaw_ori;
      m.getRPY(roll, pitch, yaw);
      roll = roll * 180.0 / 3.14159;
      pitch = pitch * 180.0 / 3.14159;
      yaw = yaw * 180.0 / 3.14159;

      imu_yaw = yaw - imu_first_yaw;
      imu_roll = roll - imu_first_roll;
      imu_pitch = pitch - imu_first_pitch;
      if(imu_yaw > 180) imu_yaw = imu_yaw -360.0;
      if(imu_yaw < -180) imu_yaw = imu_yaw +360.0;
      std::cout<<"imu roll-> "<< imu_roll <<"  imu pitch-> "<< imu_pitch <<"   imu yaw-> "<< imu_yaw << "imu_cnt->  " << imu_cnt<<std::endl; 

      tf::Quaternion qua;
      qua.setRPY(imu_roll* 3.14159/180.0, imu_pitch* 3.14159/180.0, imu_yaw* 3.14159/180.0);
      geometry_msgs::Quaternion rota_quat;
      tf::quaternionTFToMsg(qua, rota_quat);

      // const auto& imu_ori = rota_quat;
      const auto& imu_ori = (*closest_imu)->orientation;
      const auto& imu_acc = (*closest_imu)->linear_acceleration;

      geometry_msgs::Vector3Stamped acc_imu;
      geometry_msgs::Vector3Stamped acc_base;
      geometry_msgs::QuaternionStamped quat_imu;  
      geometry_msgs::QuaternionStamped quat_base;

      quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;
      quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
      acc_imu.vector = (*closest_imu)->linear_acceleration;
      quat_imu.quaternion = (*closest_imu)->orientation;
      // quat_imu.quaternion = rota_quat;
      quat_imu.quaternion.x = (*closest_imu)->orientation.x - first_imu_msg.orientation.x;
      quat_imu.quaternion.y = (*closest_imu)->orientation.y - first_imu_msg.orientation.y;
      quat_imu.quaternion.z = (*closest_imu)->orientation.z - first_imu_msg.orientation.z;
      quat_imu.quaternion.w = (*closest_imu)->orientation.w - first_imu_msg.orientation.w;
      
      // try {
      //   tf_listener.transformVector(base_frame_id, acc_imu, acc_base);
      //   tf_listener.transformQuaternion(base_frame_id, quat_imu, quat_base);
      // } catch (std::exception& e) {
      //   std::cerr << "failed to find transform!!" << std::endl;
      //   return false;
      // }

      keyframe->acceleration = Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
      keyframe->orientation = Eigen::Quaterniond(quat_imu.quaternion.w, quat_imu.quaternion.x, quat_imu.quaternion.y, quat_imu.quaternion.z);
      keyframe->orientation = keyframe->orientation;
      if(keyframe->orientation->w() < 0.0) {
        keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
      }

      if(enable_imu_orientation) { 
        std::cout<<"imu constraints......................................... " <<std::endl; 
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
        auto edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0));
      }

      if(enable_imu_acceleration) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
        g2o::OptimizableGraph::Edge* edge = graph_slam->add_se3_prior_vec_edge(keyframe->node, Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_acceleration_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_acceleration_edge_robust_kernel_size", 1.0));
      }
      updated = true;
    }

    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp,
      [=](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& imu) {
        return stamp < imu->header.stamp;
      }
    );
    imu_queue.erase(imu_queue.begin(), remove_loc);

    return true;
  }


  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(const graph_mapping_tools::FloorCoeffsConstPtr& floor_coeffs_msg) {
    if(floor_coeffs_msg->coeffs.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
    floor_coeffs_queue.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
  bool flush_floor_queue() {
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

    if(keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    for(const auto& floor_coeffs : floor_coeffs_queue) {
      if(floor_coeffs->header.stamp > latest_keyframe_stamp) {
        break;
      }

      auto found = keyframe_hash.find(floor_coeffs->header.stamp);
      if(found == keyframe_hash.end()) {
        continue;
      }

      if(!floor_plane_node) {
        floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)); // add plane node 
        floor_plane_node->setFixed(true);
      }

      const auto& keyframe = found->second;

      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("floor_edge_robust_kernel", "NONE"), private_nh.param<double>("floor_edge_robust_kernel_size", 1.0));

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp,
      [=](const ros::Time& stamp, const graph_mapping_tools::FloorCoeffsConstPtr& coeffs) {
        return stamp < coeffs->header.stamp;
      }
    );
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate map point cloud and publish it
   * @param event
   */
  void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
    if(!map_points_pub.getNumSubscribers()) {
      std::cout << "/* not subscri the map */" << std::endl;
      return;
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution); // 0.05
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    map_points_pub.publish(cloud_msg);

    // auto cloud_insideWindow = map_cloud_generator->generate_slide_window(snapshot, 0.1, 5); // 0.05
    // globalmap  = cloud_insideWindow;
    // globalmap_updated =1;
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback(const ros::WallTimerEvent& event) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated) {
      std_msgs::Header read_until;
      read_until.stamp = ros::Time::now() + ros::Duration(30, 0);
      read_until.frame_id = points_topic;
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);
    }

    if(!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() &!flush_imu_queue()) {
      return;
    }

    // loop detection
    if(enable_loop_detection) // if loop closure is anabled
    {
      std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
      for(const auto& loop : loops) {
        Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
        Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
        auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("loop_closure_edge_robust_kernel", "NONE"), private_nh.param<double>("loop_closure_edge_robust_kernel_size", 1.0));
      }
    }
    

    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes)); // insert the new_keyframes to the back of keyframes
    new_keyframes.clear();

    // optimize the pose graph
    int num_iterations = private_nh.param<int>("g2o_solver_num_iterations", 1024);
    graph_slam->optimize(num_iterations);

    // publish tf
    const auto& keyframe = keyframes.back(); // after the optimization, why the result will be saved to keyframes ?
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();
    map2odomTrans = trans; // save the trans

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(),
      [=](const KeyFrame::Ptr& k) {
        return std::make_shared<KeyFrameSnapshot>(k);
    });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();

    // publish 
    if(odom2map_pub.getNumSubscribers()) { // if the tf is subscribed, it will be published
      geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
      odom2map_pub.publish(ts);
    }

    if(markers_pub.getNumSubscribers()) { // if the markers are subscribed, it will be published
      auto markers = create_marker_array(ros::Time::now());
      markers_pub.publish(markers);
    }
  }

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) const {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(5);

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map_graph_tools";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    visualization_msgs::Marker& imu_marker = markers.markers[4];
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = 4;
    imu_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

    traj_marker.points.resize(keyframes.size());
    traj_marker.colors.resize(keyframes.size());
    for(int i=0; i<keyframes.size(); i++) {
      Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / keyframes.size();
      traj_marker.colors[i].r = 1.0 - p;
      traj_marker.colors[i].g = p;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;

      if(keyframes[i]->acceleration) {
        Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
        geometry_msgs::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();

        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.1;

        imu_marker.points.push_back(point);
        imu_marker.colors.push_back(color);
      }
    }

    // edge markers
    visualization_msgs::Marker& edge_marker = markers.markers[1];
    edge_marker.header.frame_id = "map_graph_tools";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

    auto edge_itr = graph_slam->graph->edges().begin();
    for(int i=0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
        edge_marker.colors[i*2].r = 1.0 - p1;
        edge_marker.colors[i*2].g = p1;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0 - p2;
        edge_marker.colors[i*2 + 1].g = p2;
        edge_marker.colors[i*2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i*2].z += 0.5;
          edge_marker.points[i*2 + 1].z += 0.5;
        }

        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].b = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].b = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge);
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z() + 0.5;
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z() + 0.5;

        edge_marker.colors[i*2].r = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXYZ* edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
      if(edge_priori_xyz) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xyz->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z() + 0.5;
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].r = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }
    }

    // sphere
    bool show_sphere =0;
    if(show_sphere)
    {
      visualization_msgs::Marker& sphere_marker = markers.markers[3];
      sphere_marker.header.frame_id = "map_graph_tools";
      sphere_marker.header.stamp = stamp;
      sphere_marker.ns = "loop_close_radius";
      sphere_marker.id = 0;
      sphere_marker.type = visualization_msgs::Marker::SPHERE;

      if(!keyframes.empty()) {
        Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
        sphere_marker.pose.position.x = pos.x();
        sphere_marker.pose.position.y = pos.y();
        sphere_marker.pose.position.z = pos.z();
      }
      sphere_marker.pose.orientation.w = 1.0;
      sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

      sphere_marker.color.r = 1.0;
      sphere_marker.color.a = 0.3;
    }

    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(graph_mapping_tools::DumpGraphRequest& req, graph_mapping_tools::DumpGraphResponse& res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::string directory = req.destination;

    if(directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
      std::string directory(buffer.data());
    }

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << directory << std::endl;

    graph_slam->save(directory + "/graph.g2o");
    for(int i=0; i<keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->dump(sst.str());
    }

    if(zero_utm) {
      std::ofstream zero_utm_ofs(directory + "/zero_utm");
      zero_utm_ofs << *zero_utm << std::endl;
    }

    res.success = true;
    return true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(graph_mapping_tools::SaveMapRequest& req, graph_mapping_tools::SaveMapResponse& res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate_map(snapshot, req.resolution);
    if(!cloud) {
      res.success = false;
      return true;
    }

    if(zero_utm && req.utm) {
      for(auto& pt : cloud->points) {
        pt.getVector3fMap() += (*zero_utm).cast<float>();
      }
    }

    bool ifTransform2LLH = 0; // transform point cloud into LLH for data to Huawei
    if(ifTransform2LLH)
    {
      Eigen::MatrixXd ori_llh;  //original
      ori_llh.resize(3, 1);
      ori_llh(0) = 114.16931259;
      ori_llh(1) = 22.3111737354;
      ori_llh(2) = 6;
      int pt_cnt = 0;
      FILE* fp_out_mappoints = fopen("/home/wws/points_map_in_llh.csv", "w+"); //
      if (fp_out_mappoints == NULL) {
        std::cout << "/* Error opening file!! */" << std::endl;
      } 
      for(auto& pt : cloud->points) {
        // pt.getVector3fMap() += (*zero_utm).cast<float>();
        double prex_ = pt.x;
        double prey_ = pt.y;
        double prez_ = pt.z;
        double origin_azimuth = 348.747632943 + 180 - 1.35;
        double theta = -1 * (origin_azimuth - 90)*( 3.141592 / 180.0 );
        // pt.x = (prex_ * cos(theta) - prey_ * sin(theta)) + 1.3;
        // pt.y = (prex_ * sin(theta) + prey_ * cos(theta)) + 1.3;
        Eigen::MatrixXd ENU;
        ENU.resize(3, 1);
        ENU(0) = (prex_ * cos(theta) - prey_ * sin(theta)) + 1.3;
        ENU(1) = (prex_ * sin(theta) + prey_ * cos(theta)) + 1.3;
        ENU(2) = pt.z;
        Eigen::MatrixXd ECEF = gnss_tools_.enu2ecef(ori_llh, ENU);
        Eigen::MatrixXd LLH = gnss_tools_.ecef2llh(ECEF);
        pt.x = LLH(0);
        pt.y = LLH(1); 
        pt.z = LLH(2);
        // pt.x = ENU(0);
        // pt.y = ENU(1);
        // pt.z = ENU(2);
        pt_cnt++;
        fprintf(fp_out_mappoints, "%d,%3.7f,%3.7f,%3.7f \n", pt_cnt, pt.x,pt.y,pt.z);
      }
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if(zero_utm) {
      std::ofstream ofs(req.destination + ".utm");
      ofs << (*zero_utm).transpose() << std::endl;
    }

    int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
    res.success = ret == 0;

    return true;
  }
private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::WallTimer optimization_timer;
  ros::WallTimer map_publish_timer;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>> sync;

  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;
  ros::Subscriber navsat_sub; 
  ros::Subscriber spancpt_sub;
  ros::Publisher spancpt_markers_pub;
  ros::Subscriber sub_laser_odometry_;

  ros::Subscriber imu_sub;
  ros::Subscriber floor_sub;

  ros::Publisher markers_pub;

  std::string map_frame_id;
  std::string odom_frame_id;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  ros::Publisher odom2map_pub;

  std::string points_topic;
  ros::Publisher read_until_pub;
  ros::Publisher map_points_pub;
  ros::Publisher graph_pose_pub, mapped_pose_pub;
  ros::Publisher yaw_pub;
  ros::Publisher pub_car;
  ros::Publisher pub_odom_llh; // publsih the odom in llh from LiDAR odometry

  tf::TransformListener tf_listener;

  ros::ServiceServer dump_service_server;
  ros::ServiceServer save_map_service_server;

  // keyframe queue
  std::string base_frame_id;
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  double use_span_cpt;
  double gps_time_offset;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
  boost::optional<Eigen::Vector3d> zero_utm;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;
  GNSS_Tools gnss_tools_;
  int cptspan_count = 0;
  double azimuth;
  double origin_azimuth;
  Eigen::MatrixXd originllh;
  visualization_msgs::Marker spancpt_marker;
  Eigen::MatrixXd pre_ecef;
  bool origin_set = false;

  // imu queue
  double imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::ImuConstPtr> imu_queue;
  std_msgs::Header imu_header;

  // loop detection 
  bool enable_loop_detection;

  double imu_roll=0,imu_pitch=0,imu_yaw=0, graph_pose_yaw=0, span_yaw=0;
  double imu_first_yaw=0,imu_first_roll=0,imu_first_pitch=0;
  int imu_cnt=0;

  // floor_coeffs queue
  double floor_edge_stddev;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<graph_mapping_tools::FloorCoeffsConstPtr> floor_coeffs_queue;

  // for map cloud generation
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node; // first node
  g2o::EdgeSE3* anchor_edge; // first edge
  g2o::VertexPlane* floor_plane_node;
  std::vector<KeyFrame::Ptr> keyframes; // save key frames throughout the optimization
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;

  // trans from odom to map
  Eigen::Isometry3d map2odomTrans;
  sensor_msgs::Imu first_imu_msg;

  // mapping
  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
  bool globalmap_updated =0;

  //tf 
  tf::TransformBroadcaster odom2velodyne_broadcaster;
  


};

}

PLUGINLIB_EXPORT_CLASS(graph_mapping_tools::HdlGraphSlamNodelet, nodelet::Nodelet)
