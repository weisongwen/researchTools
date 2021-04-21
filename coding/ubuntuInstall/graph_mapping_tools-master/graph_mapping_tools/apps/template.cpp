#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <vector>
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
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <std_srvs/Empty.h>
#include <hdl_graph_slam/SaveMap.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>

#include <amsi/gnss_tools.hpp>

// head file for gps 
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;


namespace hdl_graph_slam {

class HdlGraphSlamNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;

  HdlGraphSlamNodelet() {
    preX = 0;
    preY = 0;
    // initialLLH.resize(3,1);
    // initialLLH(0) =  114.19289862;
    // initialLLH(1) = 22.3181226456 ;
    // initialLLH(2) = 0;

    referencellh.resize(3, 1);
    // start point of robot
    referencellh(0) = 116.4986357;
    referencellh(1) = 39.7917481;
    referencellh(2) = 22.1009979248;

    // initial point of jd provided map
    // referencellh(0) = 116.500646;
    // referencellh(1) = 39.791704;
    // referencellh(2) = 22.1009979248;




  }
  virtual ~HdlGraphSlamNodelet() {

  }

  virtual void onInit() {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // init parameters
    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
    map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
    trans_odom2map.setIdentity();

    max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);

    //
    graph_slam.reset(new GraphSLAM());
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    nmea_parser.reset(new NmeaSentenceParser());

    gps_edge_stddev = private_nh.param<double>("gps_edge_stddev", 10000.0);
    floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0);
    initialGNSSLon = private_nh.param<double>("initialGNSSLon", 114.19095395699307);
    initialGNSSLat = private_nh.param<double>("initialGNSSLat", 22.3216976234397251);
    last_gps_edge_stamp = ros::Time(0);

    // subscribers
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/lidar_odometry", 32));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/filtered_points", 32));
    sync.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>(*odom_sub, *cloud_sub, 32));
    sync->registerCallback(boost::bind(&HdlGraphSlamNodelet::cloud_callback, this, _1, _2));
    floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 32, &HdlGraphSlamNodelet::floor_coeffs_callback, this);

    if(private_nh.param<bool>("enable_gps", true)) {
      gps_sub = nh.subscribe("/gps/geopoint", 32, &HdlGraphSlamNodelet::gps_callback, this);
      nmea_sub = nh.subscribe("/nmea_sentence", 32, &HdlGraphSlamNodelet::nmea_callback, this);
    }

    jdFix_sub =nh.subscribe("/fix", 32, &HdlGraphSlamNodelet::jdFix_callback, this); 
    //ndt_pose_sub
    ndt_pose_sub =nh.subscribe("/ndt_pose", 32, &HdlGraphSlamNodelet::ndt_pose_callback, this); 

    // publishers
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
    odom2map_pub = nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2pub", 16);
    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1);
    read_until_pub = nh.advertise<std_msgs::Header>("/hdl_graph_slam/read_until", 32);
    optimizedOdom = nh.advertise<nav_msgs::Odometry>("/optimizedOdom", 32); // the odom of the final 
    map2odomTrans = nh.advertise<nav_msgs::Odometry>("/compsensateOdom", 32); // the odom of from /map to odom 
    graphedodomPub = nh.advertise<nav_msgs::Odometry>("/graphed_odm", 32); // the odom of from /map to odom 

    initial_guess_odom_Pub = nh.advertise<nav_msgs::Odometry>("/initial_guess", 32); // the odom of from /map to odom 


    dump_service_server = nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    save_map_service_server = nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);

    GNSS_odom_ENU_pub = nh.advertise<nav_msgs::Odometry>("/GNSS_odom_ENU", 16); //

    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0); //  3.0
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
    optimization_timer = mt_nh.createTimer(ros::Duration(graph_update_interval), &HdlGraphSlamNodelet::optimization_timer_callback, this);
    map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &HdlGraphSlamNodelet::map_points_publish_timer_callback, this);
  }

private: 
  // save pose to ecef
  /**********GNSS evaluation***********/
    struct result2DPose
    {
      double UTC_Time;
      double latitude;
      double longitude;
      double heading;
    };
    struct epochResult2DPose //  2D pose in one epoch
    {
      vector<result2DPose> epochData;
    };
    epochResult2DPose epochData_;
    vector<epochResult2DPose> results; // all the epochs
    double preTime=0;
    double firstEpoch =0;
    /**********GNSS evaluation***********/

    /**********lidar odometry evaluation**********
                 Lidar system
                 1. forward: x
                 2. left:    y
                 3. up:      z

    *********lidar evaluation***********/
    Eigen::MatrixXd referencellh; // origin llh
    epochResult2DPose ndtEpochData_;
    vector<epochResult2DPose> ndtResults; // all the epochs
    double ndt_pretime=0;

    double yawBias = -144.13; // 
    double ndt_pose2ENU_angle_bias;

    double lidar_offeset_x = 0.876;
    double lidar_offeset_y = 0.319;


private:
  /**
   * @brief validate the frequency for ndt_pose
   * @input one epoch data, expected frequency
   * @output 
   */
    epochResult2DPose validateNdtposeDataFrequency(epochResult2DPose epochData_, int expectedFre)
    {
      // cout<<"input size "<<epochData_.epochData.size() <<"   bias size "<<(expectedFre - epochData_.epochData.size())<<endl;
      int additionalSize = expectedFre - epochData_.epochData.size();
      result2DPose lastEpoch = epochData_.epochData[epochData_.epochData.size()-1];

      for(int i = 0; i< additionalSize; i ++)
      {
        epochData_.epochData.push_back(lastEpoch);
      }

      double first_epoch_time = epochData_.epochData[0].UTC_Time;
      for(int i = 0; i< epochData_.epochData.size(); i ++)
      {
        epochData_.epochData[i].UTC_Time = first_epoch_time + i* 1.0/40.0;
      }
      
      // cout<<"output size "<<epochData_.epochData.size()<<endl;
      return epochData_;

    }

  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    const ros::Time& stamp = odom_msg->header.stamp;
    Eigen::Isometry3d odom = odom2isometry(odom_msg);

    Eigen::Isometry3d graphed_pose_ = map2odomTranslation *  odom; // * map2odomTranslation
    Eigen::Matrix4f graphed_pose__matrix;
    graphed_pose__matrix = graphed_pose_.matrix().cast<float>(); 


    Eigen::Quaternionf quat(graphed_pose__matrix.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();
    // cout<<"graphed_pose_(0, 3)"<<graphed_pose_(0, 3) <<"graphed_pose_(1, 3)"<<graphed_pose_(1, 3)
    // <<"graphed_pose_(2, 3)"<<graphed_pose_(2, 3)<<endl;

    // odom_trans.transform.translation.x = pose(0, 3);
    // odom_trans.transform.translation.y = pose(1, 3);
    // odom_trans.transform.translation.z = pose(2, 3);
    // odom_trans.transform.rotation = odom_quat;

    // publish pose of the final optimization
    nav_msgs::Odometry odom_;
    // odom_.header.stamp = event.current_real; //stamp
    odom_.header.stamp = odom_msg->header.stamp; //stamp
    odom_.header.frame_id = "odom";
    odom_.pose.pose.position.x = graphed_pose_(0, 3);
    odom_.pose.pose.position.y = graphed_pose_(1, 3);
    odom_.pose.pose.position.z = graphed_pose_(2, 3);

    odom_.pose.pose.orientation.w = quat.w();
    odom_.pose.pose.orientation.x = quat.x();
    odom_.pose.pose.orientation.y = quat.y();
    odom_.pose.pose.orientation.z = quat.z();

    odom_.child_frame_id = "velodyne";
    odom_.twist.twist.linear.x = 0.0;
    odom_.twist.twist.linear.y = 0.0;
    odom_.twist.twist.angular.z = 0.0;
    graphedodomPub.publish(odom_);

    // add by Weisong 
    Eigen::MatrixXd ndtpose;
    ndtpose.resize(3,1);
    ndtpose(0) = graphed_pose_(0, 3);
    ndtpose(1) = graphed_pose_(1, 3);
    ndtpose(2) = graphed_pose_(2, 3);

    

    /**********Draw Vehicle with rectangle**********
               Lidar system
               1. forward: x
               2. left:    y
               3. up:      z

  *********Draw Vehicle with rectangle***********/
    visualization_msgs::MarkerArray markers;
    

    tf::Quaternion q(
      odom_.pose.pose.orientation.x,
      odom_.pose.pose.orientation.y,
      odom_.pose.pose.orientation.z,
      odom_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw, yaw_ori;
    m.getRPY(roll, pitch, yaw);
    yaw_ori = yaw;
    yaw = yaw * 180.0 / 3.14159;
    if(yaw > 180) yaw = yaw -360.0;
    if(yaw < -180) yaw = yaw +360.0;
    yaw = yaw + yawBias;
    if(yaw > 180) yaw = yaw -360.0;
    if(yaw < -180) yaw = yaw +360.0;
    // cout<<"x-> "<<ndtpose(0) <<"y-> "<<ndtpose(1) <<"yaw-> "<<yaw *1<<endl;

    /* transform from LiDAR/GNSS receiver to Vehicle*/
    // Eigen::Translation3f tlv_ltov(0.825, 0, 0);                 // tl: translation
    // Eigen::Translation3f tlv_ltov(-0.053, 0.319, 0);                 // tl: translation
    // Eigen::AngleAxisf rot_x_btol(0, Eigen::Vector3f::UnitX());  // rot: rotation
    // Eigen::AngleAxisf rot_y_btol(0, Eigen::Vector3f::UnitY());
    // Eigen::AngleAxisf rot_z_btol(0, Eigen::Vector3f::UnitZ());
    // lv_ltov = (tlv_ltov * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

    // Eigen::Translation3f lidar_xyz(ndtpose(0), ndtpose(1) , ndtpose(2));
    // Eigen::AngleAxisf lidar_roll(roll, Eigen::Vector3f::UnitX());
    // Eigen::AngleAxisf lidar_pitch(pitch, Eigen::Vector3f::UnitY());
    // Eigen::AngleAxisf lidar_yaw(yaw, Eigen::Vector3f::UnitZ());

    // Eigen::Matrix4f gps_matrix;
    // gps_matrix = (lidar_xyz * lidar_yaw * lidar_pitch * lidar_roll) * lv_ltov;

    // ndtpose(0) = gps_matrix(0,3);
    // ndtpose(1) = gps_matrix(1,3);
    /* transform from LiDAR to Vehicle*/


    double ndt_x = ndtpose(0);
    double ndt_y = ndtpose(1);

    // ndt_x = lidar_offeset_x + ndt_x;
    // ndt_y = lidar_offeset_y + ndt_y;

    double theta = (215.92 )*( 3.141592 / 180.0 ); // theta = (215.92 )*( 3.141592 / 180.0 )
    Eigen::MatrixXd ndt_ENU; // ndt pose in ecef 
    ndt_ENU.resize(3, 1); // ndt pose in ecef
    ndt_ENU(0) = ndt_x * cos(theta) - ndt_y * sin(theta) ;
    ndt_ENU(1) = ndt_x * sin(theta) + ndt_y * cos(theta) ;
    ndt_ENU(2) = 0;



    // ndt_ENU(0) = ndt_ENU(0) + 0.825 * cos((yaw + 180) * 3.141592 / 180.0); // Lidar to vehicle
    // ndt_ENU(1) = ndt_ENU(1) + 0.825 * sin((yaw + 180) * 3.141592 / 180.0); // Lidar to vehicle


    cout<<"ndt_ENU(0)   "<<ndt_ENU(0) <<"   ndt_ENU(1)"<<ndt_ENU(1)
    <<"  ndt_ENU(2)"<<ndt_ENU(2)<<endl;

    

    Eigen::MatrixXd ndt_ecef; // ndt pose in ecef 
    ndt_ecef.resize(3, 1); // ndt pose in ecef
    ndt_ecef = gnss_tools_.enu2ecef(referencellh,ndt_ENU);
    Eigen::MatrixXd llh;
    llh.resize(3, 1);
    llh = gnss_tools_.ecef2llh(ndt_ecef);

    result2DPose singleLLH;
    singleLLH.UTC_Time = odom_msg->header.stamp.toSec(); //
    // cout<< "singleLLH.UTC_Time  "<<singleLLH.UTC_Time<<endl;
    singleLLH.longitude = llh(0);
    singleLLH.latitude = llh(1);
    singleLLH.heading = yaw;
    cout<<"yaw-> "<<yaw<<endl;

    nav_msgs::Odometry ini_gue;
    // odom_.header.stamp = event.current_real; //stamp
    ini_gue.header.stamp = odom_msg->header.stamp; //stamp
    ini_gue.header.frame_id = "odom";
    ini_gue.pose.pose.position.x = ndt_ENU(0)  + (1 * (0.17211)); // latitude
    ini_gue.pose.pose.position.y = ndt_ENU(1)  + (1 * (0.059)); // longitude 
    ini_gue.pose.pose.position.z = graphed_pose_(2, 3) + 0.22; // altitude
    yaw_ori = yaw_ori + (-2.51);

    tf::Quaternion q_ini_gue;
    q_ini_gue = tf::createQuaternionFromRPY( roll, pitch, yaw_ori );  // Create this quaternion from roll/pitch/yaw (in radians)

    // ini_gue.pose.pose.orientation = q_ini_gue;
    ini_gue.pose.pose.orientation.x =q_ini_gue[0];
    ini_gue.pose.pose.orientation.y =q_ini_gue[1];
    ini_gue.pose.pose.orientation.z =q_ini_gue[2];
    ini_gue.pose.pose.orientation.w =q_ini_gue[3];

    ini_gue.child_frame_id = "velodyne";
    ini_gue.twist.twist.linear.x = 0.0;
    ini_gue.twist.twist.linear.y = 0.0;
    ini_gue.twist.twist.angular.z = 0.0;
    initial_guess_odom_Pub.publish(ini_gue);



    ndtEpochData_.epochData.push_back(singleLLH);
    if(fabs((int(odom_msg->header.stamp.sec)) - ndt_pretime) ==1)
    {
      ndtEpochData_ = validateNdtposeDataFrequency(ndtEpochData_, 20);
      ndtResults.push_back(ndtEpochData_);
      cout<<" size of epochs-> "<< ndtResults.size() << "   size of this epoch-> "<<ndtEpochData_.epochData.size()<<endl;
      ndtEpochData_.epochData.clear();

      // SAVE TO CSV
      FILE* fp_out = fopen("/home/wenws/Downloads/submission.csv", "w+");
      for (int n=0; n<ndtResults.size(); n++)
      {
        for(int m =0; m < ndtResults[n].epochData.size(); m ++)
        {

          fprintf(fp_out, "%10.7f ,%2.10f ,%3.10f ,%3.7f\n", 
            ndtResults[n].epochData[m].UTC_Time, ndtResults[n].epochData[m].latitude, ndtResults[n].epochData[m].longitude
            , ndtResults[n].epochData[m].heading);
        }
      }
      fclose(fp_out);
    }

    // if((fix_msg->header.stamp.toSec()) >= 1542356962) // 1542356962.53
    ndt_pretime = int(odom_msg->header.stamp.sec);

    // 

    LiDAROdometry = *odom_msg;
    LiDAROdometryTime.push_back((double)LiDAROdometry.header.stamp.sec + double(LiDAROdometry.header.stamp.nsec)*1e-9);
    LiDAROdometryx.push_back(LiDAROdometry.pose.pose.position.x);
    LiDAROdometryy.push_back(LiDAROdometry.pose.pose.position.y);
    LiDAROdometryz.push_back(LiDAROdometry.pose.pose.position.z);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(!keyframe_updater->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        std_msgs::Header read_until;
        read_until.stamp = stamp + ros::Duration(30, 0);
        read_until.frame_id = "/velodyne_points";
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }

      return;
    }

    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph
   * @return if true, at least one keyframe is added to the pose graph
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
      new_keyframes.push_back(keyframe);

      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam->add_se3_node(odom);
      keyframe_hash[keyframe->stamp] = keyframe;

      if(i==0 && keyframes.empty()) {
        continue;
      }

      // add edge between keyframes
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(prev_keyframe->cloud, keyframe->cloud, relative_pose);
      graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
    }

    std_msgs::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(30, 0);
    read_until.frame_id = "/velodyne_points";
    read_until_pub.publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);

    return true;
  }


  //ndt_pose_callback  geometry_msgs::PoseStamped ndt_pose_msg
  void ndt_pose_callback(const geometry_msgs::PoseStampedConstPtr& ndt_pose_msg)
  {
    geometry_msgs::PoseStamped ndt_pose = *ndt_pose_msg;
  }


  void jdFix_callback(const sensor_msgs::NavSatFixConstPtr& fix_msg)
  {
    cout<<"jd /fix received "<<endl;
    sensor_msgs::NavSatFix navfix_ ;
    navfix_.header = fix_msg->header;
    navfix_.latitude = fix_msg->latitude;
    navfix_.longitude = fix_msg->longitude;
    navfix_.altitude = fix_msg->altitude;

    if(ini_navf.latitude == NULL)
      {
        ini_navf = navfix_;
        std::cout<<"ini_navf.header  -> "<<ini_navf.header<<std::endl;
        originllh.resize(3, 1);
        originllh(0) = navfix_.longitude;
        originllh(1) = navfix_.latitude;
        originllh(2) = navfix_.altitude;
        std::cout<<"reference longitude: "<<navfix_.longitude<<std::endl;
        std::cout<<"reference latitude: "<<navfix_.latitude<<std::endl;
      }
      originllh.resize(3, 1);
      originllh(0) = referencellh(0);
      originllh(1) = referencellh(1);
      originllh(2) = referencellh(2);
      Eigen::MatrixXd curLLh; // 
      curLLh.resize(3, 1);
      curLLh(0) = navfix_.longitude;
      curLLh(1) = navfix_.latitude;
      curLLh(2) = navfix_.altitude;

      Eigen::MatrixXd ecef; // 
      ecef.resize(3, 1);
      ecef = gnss_tools_.llh2ecef(curLLh);
      Eigen::MatrixXd eigenENU;; // 
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh,ecef);

      // trans and rotation
      double prex_ = eigenENU(0);
      double prey_ = eigenENU(1);
      double theta = (234.1 )*( 3.141592 / 180.0 ); // theta = (234.1 )*( 3.141592 / 180.0 )
      eigenENU(0) = prex_ * cos(theta) - prey_ * sin(theta) ;
      eigenENU(1) = prex_ * sin(theta) + prey_ * cos(theta) ; 

      geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
      gps_msg->header = fix_msg->header;
      gps_msg->position.latitude = 1 * eigenENU(1);
      gps_msg->position.longitude = -1 * eigenENU(0);
      gps_msg->position.altitude = fix_msg->position_covariance_type/12;

      // Eigen::Translation3f tl_btol(-0.878, -0.319, 0);                 // tl: translation
      // Eigen::AngleAxisf rot_x_btol(0, Eigen::Vector3f::UnitX());  // rot: rotation
      // Eigen::AngleAxisf rot_y_btol(0, Eigen::Vector3f::UnitY());
      // Eigen::AngleAxisf rot_z_btol(0, Eigen::Vector3f::UnitZ());
      // tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

      // Eigen::Translation3f gps_xyz(gps_msg->position.latitude, gps_msg->position.longitude, gps_msg->position.altitude);
      // Eigen::AngleAxisf gps_roll(0, Eigen::Vector3f::UnitX());
      // Eigen::AngleAxisf gps_pitch(0, Eigen::Vector3f::UnitY());
      // Eigen::AngleAxisf gps_yaw(0, Eigen::Vector3f::UnitZ());

      // Eigen::Matrix4f gps_matrix;
      // gps_matrix = (gps_xyz * gps_yaw * gps_pitch * gps_roll) * tf_btol;

      // gps_msg->position.latitude = gps_matrix(0,3);
      // gps_msg->position.longitude = gps_matrix(1,3);
      // gps_msg->position.altitude = fix_msg->position_covariance_type/12;


      cout<< "x-> "<< gps_msg->position.latitude <<"    y-> "<< gps_msg->position.longitude<<endl;

      if(((fix_msg->status.status) == 4) &&( (fix_msg->position_covariance_type)  > 12)) // 12
      {
        gps_callback(gps_msg); 
      }
      // gps_callback(gps_msg);
      std::cout<<"push back message to gps_queue..."<<std::endl;

      // publish the GNSS odom in LiDAR coordiante system
      nav_msgs::Odometry GNSS_odom_enu;
      GNSS_odom_enu.header.stamp = fix_msg->header.stamp;
      GNSS_odom_enu.header.frame_id = "map";

      //GNSS_odom_enu.pose.pose.position.x = eigenENU(0);
      //GNSS_odom_enu.pose.pose.position.y = eigenENU(1);

      GNSS_odom_enu.pose.pose.position.x = 1 * eigenENU(1);
      GNSS_odom_enu.pose.pose.position.y = -1 * eigenENU(0);

      GNSS_odom_enu.pose.pose.position.z = 0;
      GNSS_odom_enu.child_frame_id = "base_link";
      GNSS_odom_enu.twist.twist.linear.x = 0.0;
      GNSS_odom_enu.twist.twist.linear.y = 0.0;
      GNSS_odom_enu.twist.twist.angular.z = 0.0;
      GNSS_odom_ENU_pub.publish(GNSS_odom_enu);
  }

  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    std::vector<std::string> str_vec_ptr;
    std::string token;
    std::stringstream ss(nmea_msg->sentence);
    bool find_SOL_COMPUTED =0;
    while (getline(ss, token, ' '))
    {
      if(token == "SOL_COMPUTED") // solutions are computed 
      {
        // std::cout<<"message obtained"<<std::endl;
        find_SOL_COMPUTED = true;
      }
      if( find_SOL_COMPUTED ) // find flag SOL_COMPUTED
      {
        str_vec_ptr.push_back(token);
      }
    }
        if(find_SOL_COMPUTED)
    {
      sensor_msgs::NavSatFix navfix_ ;
      navfix_.header = nmea_msg->header;
      std::cout << std::setprecision(17);
      double lat = strtod((str_vec_ptr[2]).c_str(), NULL);
      double lon = strtod((str_vec_ptr[3]).c_str(), NULL);
      double alt = strtod((str_vec_ptr[4]).c_str(), NULL);
      std::cout << std::setprecision(17);

      navfix_.latitude = lat;
      navfix_.longitude = lon;
      navfix_.altitude = alt;
      if(ini_navf.latitude == NULL)
      {
        ini_navf = navfix_;
        std::cout<<"ini_navf.header  -> "<<ini_navf.header<<std::endl;
        originllh.resize(3, 1);
        originllh(0) = navfix_.longitude;
        originllh(1) = navfix_.latitude;
        originllh(2) = navfix_.altitude;
        std::cout<<"reference longitude: "<<navfix_.longitude<<std::endl;
        std::cout<<"reference latitude: "<<navfix_.latitude<<std::endl;
      }
      Eigen::MatrixXd curLLh; // 
      curLLh.resize(3, 1);
      curLLh(0) = navfix_.longitude;
      curLLh(1) = navfix_.latitude;
      curLLh(2) = navfix_.altitude;

      Eigen::MatrixXd ecef; // 
      ecef.resize(3, 1);
      ecef = gnss_tools_.llh2ecef(curLLh);
      Eigen::MatrixXd eigenENU;; // 
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh,ecef);

      // trans and rotation
      double prex_ = eigenENU(0);
      double prey_ = eigenENU(1);
      double theta = (68.5 )*( 3.141592 / 180.0 ); // Berkeley CPT
      eigenENU(0) = prex_ * cos(theta) - prey_ * sin(theta) ;
      eigenENU(1) = prex_ * sin(theta) + prey_ * cos(theta) ; 

      geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
      gps_msg->header = nmea_msg->header;
      gps_msg->position.latitude = 1 * eigenENU(1);
      gps_msg->position.longitude = -1 * eigenENU(0);
      gps_msg->position.altitude = eigenENU(2);

      gps_callback(gps_msg);
      std::cout<<"push back message to gps_queue..."<<std::endl;
    }
  }

  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::GeoPointStampedConstPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_queue.push_back(gps_msg);
  }

  std::ofstream ofs;

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    auto seek = keyframes.begin();
    for(const auto& gps_msg : gps_queue) {
      seek = std::lower_bound(seek, keyframes.end(), gps_msg->header.stamp, [&](const KeyFrame::Ptr& key, const ros::Time& stamp) { return key->stamp < stamp; });
      if(seek == keyframes.end()) {
        break;
      }

      double  residual = ((*seek)->stamp - gps_msg->header.stamp).toSec(); // GPS-Keyframe and Gps frame time bias difference  
      std::cout << "residual---------------------:" << residual<< std::endl;
      if(std::abs(residual) > 1.25 || (*seek)->utm_coord) { // 0.25 
        continue;
      }

      if(gps_msg->header.stamp - last_gps_edge_stamp < ros::Duration(1.0)   ) { // constant update with GPS initially 30
        continue;
      }

      Eigen::MatrixXd enu_; // the enu for output
      enu_.resize(3, 1);
      double E_ = double(gps_msg->position.latitude); // this is the E in ENU coordinate system
      double N_ = double(gps_msg->position.longitude); // this is the N in ENU coordinate system
      double Covariance = double(gps_msg->position.altitude); // this is the positioning covariance of GNSS positioning
      enu_(0) = E_;
      enu_(1) = N_;
      enu_(2) = double(gps_msg->position.altitude);
      Eigen::Vector3d xyz(enu_(0), enu_(1), enu_(2)); // save the 
      double prex_ = xyz(0);
      double prey_ = xyz(1);

      pt2GPS_ = xyz;
      (*seek)->utm_coord = xyz;

      // Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity() / (gps_edge_stddev ); // fixed : 13 3D case
      // Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / (gps_edge_stddev ); // fixed : 13  2D case
      // gps_msg->position.altitude
      Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / (gps_edge_stddev/gps_msg->position.altitude ); // fixed : 13  2D case
      if(Covariance<3200000000) //  for open loop datasheet: set threshold 30            for closed loop datasheet: set threshold 40
      {
        // graph_slam->add_se3_prior_xyz_edge((*seek)->node, xyz.head<3>(), information_matrix); // 3D position added into graph_slam
        graph_slam->add_se3_prior_xy_edge((*seek)->node, xyz.head<2>(), information_matrix); // 2D position added into graph_slam
        std::cout << "xyz(0):" << xyz(0)<< std::endl;
        std::cout << "xyz(1):" << xyz(1)<< std::endl;
        std::cout << "xyz(2):" << xyz(2)<< std::endl;

      }
      
      last_gps_edge_stamp = gps_msg->header.stamp;

      updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), latest_keyframe_stamp,
      [=](const ros::Time& stamp, const geographic_msgs::GeoPointStampedConstPtr& geopoint) {
        return stamp < geopoint->header.stamp;
      }
    );
    gps_queue.erase(gps_queue.begin(), remove_loc);

    return updated;
  }


/*
author: WEN Weisong (17902061r@connect.polyu.hk)
function: llh to ecef
input: llh (Matrix3d)
output: ecef (Matrix3d)
*/
Eigen::MatrixXd llh2ecef(Eigen::MatrixXd data) // transform the llh to ecef
{
  Eigen::MatrixXd ecef; // the ecef for output
  ecef.resize(3, 1);
  double a = 6378137.0;
  double b = 6356752.314;
  double n, Rx, Ry, Rz;
  double lon = (double)data(0) * 3.1415926 / 180.0; // lon to radis
  double lat = (double)data(1) * 3.1415926 / 180.0; // lat to radis
  double alt = (double)data(2); // altitude
  n = a * a / sqrt(a * a * cos(lat) * cos(lat) + b * b * sin(lat) * sin(lat));
  Rx = (n + alt) * cos(lat) * cos(lon);
  Ry = (n + alt) * cos(lat) * sin(lon);
  Rz = (b * b / (a * a) * n + alt) * sin(lat);
  ecef(0) = Rx; // return value in ecef
  ecef(1) = Ry; // return value in ecef
  ecef(2) = Rz; // return value in ecef
  return ecef;

  /**************for test purpose*************************
  Eigen::MatrixXd llh;
  llh.resize(3, 1);
  Eigen::MatrixXd ecef;
  ecef.resize(3, 1);
  llh(0) = 114.1772621294604;
  llh(1) = 22.29842880200087;
  llh(2) = 58;
  ecef = llh2ecef(llh);
  cout << "ecef ->: " << ecef << "\n";
  */
}

/*
author: WEN Weisong (17902061r@connect.polyu.hk)
function: ecef to enu
input: original llh, and current ecef (Matrix3d)
output: enu (Matrix3d)
*/
Eigen::MatrixXd ecef2enu(Eigen::MatrixXd originllh, Eigen::MatrixXd ecef) // transform the ecef to enu 
{
  double pi = 3.1415926; // pi 
  double DEG2RAD = pi / 180.0;
  double RAD2DEG = 180.0 / pi;

  Eigen::MatrixXd enu; // the enu for output
  enu.resize(3, 1); // resize to 3X1
  Eigen::MatrixXd oxyz; // the original position 
  oxyz.resize(3, 1); // resize to 3X1

  double x, y, z; // save the x y z in ecef
  x = ecef(0);
  y = ecef(1);
  z = ecef(2);

  double ox, oy, oz; // save original reference position in ecef
  oxyz = llh2ecef(originllh);
  ox = oxyz(0); // obtain x in ecef 
  oy = oxyz(1); // obtain y in ecef
  oz = oxyz(2); // obtain z in ecef

  double dx, dy, dz;
  dx = x - ox;
  dy = y - oy;
  dz = z - oz;

  double lonDeg, latDeg, _; // save the origin lon alt in llh
  lonDeg = originllh(0);
  latDeg = originllh(1);
  double lon = lonDeg * DEG2RAD;
  double lat = latDeg * DEG2RAD;

  //save ENU
  enu(0) = -sin(lon) * dx + cos(lon) * dy;
  enu(1) = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
  enu(2) = cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
  return enu;

  /**************for test purpose*****suqare distance is about 37.4 meters********************
  Eigen::MatrixXd llh;  //original
  llh.resize(3, 1);
  llh(0) = 114.1775072541416;
  llh(1) = 22.29817969722738;
  llh(2) = 58;
  Eigen::MatrixXd ecef;
  ecef.resize(3, 1);
  ecef(0) = -2418080.9387265667;
  ecef(1) = 5386190.3905763263;
  ecef(2) = 2405041.9305451373;
  Eigen::MatrixXd enu;
  enu.resize(3, 1);
  enu = ecef2enu(llh, ecef);
  cout << "enu ->: " << enu << "\n";
  */
}

  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(const hdl_graph_slam::FloorCoeffsConstPtr& floor_coeffs_msg) {
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

      const auto& keyframe = found->second;

      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      graph_slam->add_se3_plane_edge(keyframe->node, graph_slam->floor_plane_node, coeffs, information);

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp,
      [=](const ros::Time& stamp, const hdl_graph_slam::FloorCoeffsConstPtr& coeffs) {
        return stamp < coeffs->header.stamp;
      }
    );
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate a map point cloud and publish it
   * @param event
   */
  void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
    // if(!map_points_pub.getNumSubscribers()) {
    //   return;
    // }     

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, 0.05);
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    map_points_pub.publish(cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    if(!flush_keyframe_queue() & !flush_floor_queue() & !flush_gps_queue()) {
      std_msgs::Header read_until;
      read_until.stamp = event.current_real + ros::Duration(30, 0);
      read_until.frame_id = "/velodyne_points";
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);

      return;
    }

    // loop detection
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    for(const auto& loop : loops) {
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
      graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
    }

    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // optimize the pose graph
    graph_slam->optimize();

    // publish tf
    const auto& keyframe = keyframes.back();
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    map2odomTranslation = trans; // record the trans (Weiosng. )

    cout<<"translation().x"<<keyframe->node->estimate().translation().x()<<endl;

    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    // publish pose of the final optimization
    nav_msgs::Odometry odom_;
    nav_msgs::Odometry comOdom;
    // odom_.header.stamp = event.current_real; //stamp
    odom_.header.stamp = keyframe->stamp; //stamp
    odom_.header.frame_id = "velodyne";

    Eigen::Vector3d pos = keyframe->node->estimate().translation(); // this is 


    double graph_time = ((double)odom_.header.stamp.sec + double(odom_.header.stamp.nsec)*1e-9); // time stamp of the keyframe
    double bias =100000;
    int index_ =0;
    for(int index= 0; index<LiDAROdometryTime.size(); index++)
    {
      if(fabs (graph_time - LiDAROdometryTime[index]) <bias)
      {
        index_ = index;
        bias = fabs (graph_time - LiDAROdometryTime[index]);
      }
      
    }
    odom_.pose.pose.position.x = pos.x() - LiDAROdometryx[index_];
    odom_.pose.pose.position.y = pos.y() - LiDAROdometryy[index_];
    odom_.pose.pose.position.z = pos.z() - LiDAROdometryz[index_];
    std::cout << "time bias:" << fabs (graph_time - LiDAROdometryTime[index_])  << std::endl; 

    odom_.child_frame_id = "map";
    odom_.twist.twist.linear.x = 0.0;
    odom_.twist.twist.linear.y = 0.0;
    odom_.twist.twist.angular.z = 0.0;

    // optimizedOdom.publish(odom_);


    if(map_points_pub.getNumSubscribers()) {
      std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
      std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(),
        [=](const KeyFrame::Ptr& k) {
          return std::make_shared<KeyFrameSnapshot>(k);
      });

      std::lock_guard<std::mutex> lock(keyframes_snapshot_mutex);
      keyframes_snapshot.swap(snapshot);
    }

//     if(odom2map_pub.getNumSubscribers()) {
    if(1) {
      geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
      odom2map_pub.publish(ts);
      comOdom.pose.pose.position.x = ts.transform.translation.x;
      comOdom.pose.pose.position.y = ts.transform.translation.y;
      comOdom.pose.pose.position.z = ts.transform.translation.z;

      comOdom.child_frame_id = "map";
      comOdom.twist.twist.linear.x = 0.0;
      comOdom.twist.twist.linear.y = 0.0;
      comOdom.twist.twist.angular.z = 0.0;
      map2odomTrans.publish(comOdom);
    }

    if(markers_pub.getNumSubscribers()) {
      auto markers = create_marker_array(event.current_real);
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
    markers.markers.resize(4);

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 2; // initially 0.5

    traj_marker.points.resize(keyframes.size());
    traj_marker.colors.resize(keyframes.size());
    for(int i=0; i<keyframes.size(); i++) {  // Node
      Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / keyframes.size();
//       traj_marker.colors[i].r = 1.0 - p;
//       traj_marker.colors[i].g = p;
      traj_marker.colors[i].r = 1.0;
      traj_marker.colors[i].g = 0.0;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;
      if(i == (keyframes.size() -1))
      {
         // cout<<"translation().x"<<keyframes[i]->node->estimate().translation().x()<<endl;
      }
    } 
   

    // edge markers
    visualization_msgs::Marker& edge_marker = markers.markers[1];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.9; // initially 0.05

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
        edge_marker.colors[i*2].r = 0.0 ;
        edge_marker.colors[i*2].g = 0.0;
	edge_marker.colors[i*2].b = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 0;
        edge_marker.colors[i*2 + 1].g = 0;
	edge_marker.colors[i*2 + 1].b = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i*2].z += 0.5;
          edge_marker.points[i*2 + 1].z += 0.5;
        }

        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge); // Plane 
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

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge); // GPS
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();
        // std::cout << "pt2(0)->:" << pt2(0)<< std::endl;
        // std::cout << "pt2(1)->:" << pt2(1)<< std::endl;

        edge_marker.colors[i*2].r = 1.0;
	      edge_marker.colors[i*2].g = 0.0;
        edge_marker.colors[i*2].b = 0.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
	      edge_marker.colors[i*2 + 1].g = 0.0;
	      edge_marker.colors[i*2 + 1].b = 0.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }
    }

    // sphere for loop closure 
    // visualization_msgs::Marker& sphere_marker = markers.markers[3];
    // sphere_marker.header.frame_id = "map";
    // sphere_marker.header.stamp = stamp;
    // sphere_marker.ns = "loop_close_radius";
    // sphere_marker.id = 0;
    // sphere_marker.type = visualization_msgs::Marker::SPHERE;

    // if(!keyframes.empty()) {
    //   Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
    //   sphere_marker.pose.position.x = pos.x();
    //   sphere_marker.pose.position.y = pos.y();
    //   sphere_marker.pose.position.z = pos.z();
    // }
    // sphere_marker.pose.orientation.w = 1.0;
    // sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

    // sphere_marker.color.r = 1.0;
    // sphere_marker.color.a = 0.3;

    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::array<char, 64> buffer;
    buffer.fill(0);
    time_t rawtime;
    time(&rawtime);
    const auto timeinfo = localtime(&rawtime);
    strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    std::string directory(buffer.data());

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << std::flush;
    system("pwd");

    graph_slam->save(directory + "/graph.g2o");
    for(int i=0; i<keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->dump(sst.str());
    }

    return true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(hdl_graph_slam::SaveMapRequest& req, hdl_graph_slam::SaveMapResponse& res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
    if(!cloud) {
      res.success = false;
      return true;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
    res.success = ret == 0;

    return true;
  }
private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::Timer optimization_timer;
  ros::WallTimer map_publish_timer;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>> sync;

  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;
  ros::Subscriber floor_sub;

  ros::Subscriber jdFix_sub; // subscribe the /fix topic from jindong dataset  
  ros::Subscriber ndt_pose_sub; // subscribe the /bdt_pose from matching 
  ros::Publisher GNSS_odom_ENU_pub; //

  ros::Publisher markers_pub;

  std::string map_frame_id;
  std::string odom_frame_id;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  ros::Publisher odom2map_pub;

  ros::Publisher read_until_pub;
  ros::Publisher map_points_pub;
  ros::Publisher optimizedOdom;
  ros::Publisher map2odomTrans;
  ros::Publisher graphedodomPub;
  ros::Publisher initial_guess_odom_Pub;

  ros::ServiceServer dump_service_server;
  ros::ServiceServer save_map_service_server;

  // keyframe queue
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  double gps_edge_stddev;
  double preX;
  double preY;
  Eigen::Vector3d pt2GPS_ ;
  boost::optional<Eigen::Vector3d> zero_utm;
  boost::optional<Eigen::Vector3d> zero_utm_;
  ros::Time last_gps_edge_stamp;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;

  double initialGNSSLon; // initial gps longitude 
  double initialGNSSLat; // initial gps latitude
  Eigen::MatrixXd initialLLH; // the ecef for output
  nav_msgs::Odometry LiDAROdometry; // LiDAR odometry information
  std::vector <double> LiDAROdometryTime;
  std::vector <double> LiDAROdometryx;
  std::vector <double> LiDAROdometryy;
  std::vector <double> LiDAROdometryz;

  // gps sentence
  sensor_msgs::NavSatFix ini_navf ; // initial sensor msg
  // gnss_tools
  GNSS_Tools gnss_tools_;
  Eigen::MatrixXd originllh; // origin llh

  Eigen::Matrix4f tf_btol, tf_ltob,lv_ltov;



  // floor_coeffs queue
  double floor_edge_stddev;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

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

  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;

  // for increase final pose frequecny, record the 
  Eigen::Isometry3d map2odomTranslation;

  

};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet)
