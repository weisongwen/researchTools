# synchronize, allign, simultaneously subscribe several ros topics using messages filter
sometimes, we want to subscribe several topics and synchronize them. This
- step 1: define message_filters::Subscriber
```
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
```
```
/* To subscribe the gnss raw msg and doppler measurements at the same time 
    Reference: https://github.com/koide3/hdl_graph_slam/blob/master/apps/hdl_graph_slam_nodelet.cpp#L107
    */
    std::unique_ptr<message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>> gnss_raw_array_sub;

    std::unique_ptr<message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>> station_gnss_raw_array_sub;

    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> doppler_sub;

    std::unique_ptr<message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw;

```
- step 2: setup the subscribed ros topic
```
gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/rtk_estimator/GNSS_RTKLIB", 256));
station_gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/rtk_estimator/GNSS_station", 256)); // measurements from station
doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/rtk_estimator/velocity_from_doppler", 32));
``` 

- step 3: define call_back function 
```
syncdoppler2GNSSRaw->registerCallback(boost::bind(&pdrtk_GMM::gnssraw_doppler_msg_callback,this, _1, _2, _3));
```

- step 4: define the callback function 
```
    /**
   * @brief gnss raw msg (user end), gnss raw msg (station end) and doppler msg callback
   * @param gnss raw msg (user end), gnss raw msg (station end) and doppler msg callback
   * @return void
   @ 
   */
    void gnssraw_doppler_msg_callback(const nlosExclusion::GNSS_Raw_ArrayConstPtr& gnss_msg, const nlosExclusion::GNSS_Raw_ArrayConstPtr& station_gnss_msg, const nav_msgs::OdometryConstPtr& doppler_msg)
    {
        m_gnss_raw_mux.lock();
        gnss_frame++;
        double time0 = gnss_msg->GNSS_Raws[0].GNSS_time;
        double time1 = station_gnss_msg->GNSS_Raws[0].GNSS_time;
        double time_frame = doppler_msg->pose.pose.position.x;
        #if 1 // debug time frame
        std::cout<<"gnss time0 " <<time0 <<std::endl; 
        std::cout<<"doppler time_frame " <<time_frame <<std::endl;
        std::cout<<"station time1 " <<time1 <<std::endl;
        #endif
        if(gnss_msg->GNSS_Raws.size())
        {
            gnss_raw_buf.push(gnss_msg); 
            doppler_map[time_frame] = *doppler_msg;
            gnss_raw_map[time_frame] = *gnss_msg;
            station_gnss_raw_map[time_frame] = *station_gnss_msg;
            m_GNSS_Tools.checkRepeating(*gnss_msg);
            // Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare_GPS(
            //                                 m_GNSS_Tools.getAllPositions(*gnss_msg),
            //                                 m_GNSS_Tools.getAllMeasurements(*gnss_msg),
            //                                 *gnss_msg);
            Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                    m_GNSS_Tools.getAllPositions(*gnss_msg),
                                    m_GNSS_Tools.getAllMeasurements(*gnss_msg),
                                    *gnss_msg, "WLS");
            Eigen::Matrix<double ,3,1> ENU;
            // Eigen::Matrix<double, 3,1> ENU_ref;
            // ENU_ref<< 114.179000972, 22.3011535667, 0;
            ENU = m_GNSS_Tools.ecef2enu(ENU_ref, eWLSSolutionECEF);
            // LOG(INFO) << "ENU WLS -> "<< std::endl << ENU;
            // std::cout << "eWLSSolutionECEF"<<eWLSSolutionECEF<<std::endl;

            nav_msgs::Odometry odometry;
            // odometry.header = pose_msg->header;
            odometry.header.frame_id = "map";
            odometry.child_frame_id = "map";
            odometry.pose.pose.position.x = ENU(0);
            odometry.pose.pose.position.y = ENU(1);
            odometry.pose.pose.position.z = ENU(2);
            pub_WLS.publish(odometry);
        }
        m_gnss_raw_mux.unlock();

    }
```

This application can also be found in [gnss_rtk library](https://github.com/weisongwen/gnss_rtk/blob/master/global_fusion/src/rtk_estimator/pdrtk_GMM.cpp#L124)

### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)