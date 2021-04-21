# graph_mapping_tools
***graph_mapping_tools*** is a ROS package for 6DOF mapping using multiple sensors inclduing GNSS/INS/LIDAR. It is based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection. It also supports several graph constraints, such as GPS, IMU acceleration (gravity vector), IMU orientation (magnetic sensor), and floor plane (detected in a point cloud).  


## Nodelets
***graph_mapping_tools*** consists of four nodelets.

- *prefiltering_nodelet*
- *scan_matching_odometry_nodelet*
- *floor_detection_nodelet*
- *graph_mapping_tools_nodelet*

The input point cloud is first downsampled by *prefiltering_nodelet*, and then passed to the next nodelets. While *scan_matching_odometry_nodelet* estimates the sensor pose by iteratively applying a scan matching between consecutive frames (i.e., odometry estimation), *floor_detection_nodelet* detects floor planes by RANSAC. The estimated odometry and the detected floor planes are sent to *graph_mapping_tools*. To compensate the accumulated error of the scan matching, it performs loop detection and optimizes a pose graph which takes various constraints into account.


## Constraints (Edges)

You can enable/disable each constraint by changing params in the launch file, and you can also change the weight (\*_stddev) and the robust kernel (\*_robust_kernel) of each constraint.

- ***Odometry***

- ***Loop closure***

- ***GPS***
  - */gps/geopoint* (geographic_msgs/GeoPointStamped)
  - */gps/navsat* (sensor_msgs/NavSatFix)
  - */gpsimu_driver/nmea_sentence* (nmea_msgs/Sentence)

graph_mapping_tools supports several GPS message types. All the supported types contain (latitude, longitude, and altitude). graph_mapping_tools converts them into [the UTM coordinate](http://wiki.ros.org/geodesy), and adds them into the graph as 3D position constraints. If altitude is set to NaN, the GPS data is treated as a 2D constrait. GeoPoint is the most basic one, which consists of only (lat, lon, alt). Although NavSatFix provides many information, we use only (lat, lon, alt) and ignore all other data. If you're using HDL32e, you can directly connect *graph_mapping_tools* and *velodyne_driver* via */gpsimu_driver/nmea_sentence*.

- ***IMU acceleration (gravity vector)***
  - */gpsimu_driver/imu_data* (sensor_msgs/Imu)

This constraint rotates each pose node so that the acceleration vector associated with the node will be vertical (as the gravity vector). This is useful to compensate the accumulated tilt rotation error of the scan matching. Since we ignore acceleration by sensor motion, you should not give a big weight for this constraint.

- ***IMU orientation (magnetic sensor)***
  - */gpsimu_driver/imu_data* (sensor_msgs/Imu)

  If your IMU has a reliable magnetic orientation sensor, you can add orientation data to the graph as 3D rotation constraints. Note that, magnetic orientation sensors can be affected by external magnetic disturbances. In such cases, this constraint should be disabled.

- ***Floor plane***
  - */floor_detection/floor_coeffs* (graph_mapping_tools/FloorCoeffs)

This constraint optimizes the graph so that the floor planes (detected by RANSAC) of the pose nodes will be the same. This is designed to compensate the accumulated rotation error of the scan matching in large flat indoor environments.

## enable span-cpt
set the following true
```
  <arg name="enable_gps" default="false" />
```
## enable xsense mti 10 imu
set the following true (directly add the orientation into the factor graph)

```
  <arg name="enable_imu_acc" default="false" />
  <arg name="enable_imu_ori" default="false" />

  <param name="imu_orientation_edge_stddev" value="0.6" />
  ```
set the following true (directly add the orientation into NDT scan matching initial guess)

```
  <param name="use_imu_ori_predi" value="0.0" />
```

## Parameters
All the configurable parameters are listed in *launch/graph_mapping_tools.launch* as ros params.

## Services
- */graph_mapping_tools/dump*  (graph_mapping_tools/DumpGraph)
  - save all the data (point clouds, floor coeffs, odoms, and pose graph) to a directory.
- */graph_mapping_tools/save_map*  (graph_mapping_tools/SaveMap)
  - save the generated map as a PCD file.

rosservice call /graph_mapping_tools/save_map "resolution: 0.2
destination: '/home/wws/mapfile20190806.pcd'"

## Requirements
***graph_mapping_tools*** requires the following libraries:

- OpenMP
- PCL 1.7
- g2o
- suitesparse

The following ROS packages are required:

- geodesy
- nmea_msgs
- pcl_ros
- [ndt_omp](https://github.com/koide3/ndt_omp)

```bash
# for indigo
sudo apt-get install ros-indigo-geodesy ros-indigo-pcl_ros ros-indigo-nmea-msgs
# for kinetic
sudo apt-get install ros-kinetic-geodesy ros-kinetic-pcl_ros ros-kinetic-nmea-msgs ros-kinetic-libg2o
# for melodic
sudo apt-get install ros-melodic-geodesy ros-melodic-pcl_ros ros-melodic-nmea-msgs ros-melodic-libg2o

cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
```

Note that, in case use are using ros indigo, ***graph_mapping_tools*** cannot be built with the ros g2o binaries (ros-indigo-libg2o). ~~Install the latest g2o:~~
The latest g2o causes segfault. Use commit *a48ff8c42136f18fbe215b02bfeca48fa0c67507* instead of the latest one:

```bash
sudo apt-get install libsuitesparse-dev
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout a48ff8c42136f18fbe215b02bfeca48fa0c67507
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j8
sudo make install
```

**[optional]** *bag_player.py* script requires ProgressBar2.
```bash
sudo pip install ProgressBar2
```

## Test NDT + ground constraint + loop closure -> non-linear optimization


```bash
rosparam set use_sim_time true
roslaunch graph_mapping_tools graph_mapping_tools.launch
```

```bash
roscd graph_mapping_tools/rviz
rviz -d graph_mapping_tools.rviz
```

```bash
rosbag play --clock hdl_400.bag
```
## Test LOAM + ground constraint + loop closure -> non-linear optimization

non-linear optimization framework
```bash
rosparam set use_sim_time true
roslaunch graph_mapping_tools graph_mapping_tools.launch
```

```bash
roslaunch laserOdometry 32_scans_test.launch
```

```bash
/Downloads/jlt20190530/xiaobai$ 
rosbag play -r 0.5  2019-05-30-16-02-14.bag --clock
```

## Test LEGO-LOAM + ground constraint + loop closure -> non-linear optimization

non-linear optimization framework
```bash
rosparam set use_sim_time true
roslaunch graph_mapping_tools graph_mapping_tools.launch
```

```bash
pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5); // USE NDT changed this to laser_odom_to_init in line 204 of featureAssociation.cpp
roslaunch laserOdometry 32_scans_test.launch
```

```bash
/Downloads/jlt20190530/xiaobai$ 
rosbag play -r 0.5  2019-05-30-16-02-14.bag --clock
```

## Use graph_mapping_tools in your system

1. Define the transformation between your sensors (LIDAR, IMU, GPS) and the base of your system using static_transform_publisher (see line #11, graph_mapping_tools.launch). All the sensor data will be transformed into the common base frame, and then passed to the SLAM algorithm.

2. Remap the point cloud topic of ***prefiltering_nodelet***. Like: 
    
```bash
  <node pkg="nodelet" type="nodelet" name="prefiltering_nodelet" ...
    <remap from="/velodyne_points" to="/rslidar_points"/>
  ...
```

## Common Problems

### graph_mapping_tools_nodelet causes memory error

It has been reported that *graph_mapping_tools_nodelet* causes a memory error in some environments. I found that this is caused by a variable (*color*) in g2o::VertexPlane. Since this field is used for only visualization, we can remove it from vertex_plane.h and vertex_plane.cpp in g2o. I made a clone repository of g2o, in which I just removed it
 from the commit *a48ff8c42136f18fbe215b02bfeca48fa0c67507* of g2o. If you face this memory error problem, try to install it instead of the original g2o repository. Do not forget to checkout *graph_mapping_tools* branch.

```bash
git clone https://github.com/koide3/g2o.git
cd g2o
git checkout graph_mapping_tools
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j8
sudo make install
```

# Credites
This NDT-omp, baseline framework is heavily derived from [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam). If there is any thing inappropriate, please contact me through 17902061r@connect.polyu.hk (Weisong WEN).

## LICENSE
### BSD License – PolyU

Copyright (c) 2018 [Weisong WEN](https://weisongwen.wixsite.com/weisongwen)

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the <organization> nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
