# Paper Review

## Abstract
Expect to read a paper per day.

## Content
### GNSS

- **GNSS Overall Technology**
    - [RTK: Where Every GNSS Matters](https://www.unoosa.org/pdf/icg/2014/wg/wgb10.pdf)
    - [GNSS Technique](gnss_technique/README.md)

- **GNSS NLOS/Multipath**
    - [Robust Positioning in the Presence of Multipath and NLOS Signals ](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20190034171.pdf) 

- **GNSS Integrity**
    - [Integrity of Laser-Based Feature Extraction and Data Association ](https://ieeexplore.ieee.org/document/7479746) 
    - [Continuity Risk of Feature Extraction for Laser-Based Navigation](https://pdfs.semanticscholar.org/0959/b88975a369b9685357a710e2cfabfed86441.pdf?_ga=2.30687740.2123690660.1578646651-472082014.1564729936) 
    - [Kalman Filter-based INS Monitor to Detect GNSS Spoofers Capable of Tracking Aircraft Position](https://ieeexplore.ieee.org/document/7479805) 
    - [Integrity for GPS/LiDAR Fusion Utilizing a RAIM Framework](https://pdfs.semanticscholar.org/07fb/1f3a1b19a48d24bd78de37d40f6c14cee3e1.pdf?_ga=2.68308434.2123690660.1578646651-472082014.1564729936) 
    - [Snapshot Residual and Kalman Filter based Fault Detection and Exclusion Schemes for Robust Railway Navigation](https://ieeexplore.ieee.org/document/7954171) 
    - [Towards Navigation Safety for Autonomous Cars](https://www.insidegnss.com/auto/novdec17-JOERGER.pdf) 
    

- **GNSS RTK**

    - **Tokyo University, Prof. Kubo**
        - [Precise and Robust RTK-GNSS Positioning in Urban Environments with Dual-Antenna Configuration](https://www.mdpi.com/1424-8220/19/16/3586)
        - [RTKLIB: An Open Source Program Package for GNSS Positioning](https://ieeexplore.ieee.org/document/7479746) [[Github](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3)] [[Manual](http://www.rtklib.com/prog/manual_2.4.2.pdf)] [[Tutorial Demo](http://www.rtklib.com/rtklib_tutorial.htm)]

    - **Tsinghua University, Prof. Lu mingquan**
        - [Precise and Robust RTK-GNSS Positioning in Urban Environments with Dual-Antenna Configuration](https://www.mdpi.com/1424-8220/19/16/3586)
    
    - **Wuhan University, Prof. Li Bijun**
        - [A Vision-assisted GNSS-RTK Ambiguity FixingMethod Considering Visual Lane Line Ambiguity](https://www.ion.org/plans/upload/PLANS20Program.pdf)
    
    - **Tongji University, TBD**
        - [A Vision-assisted GNSS-RTK Ambiguity FixingMethod Considering Visual Lane Line Ambiguity](https://www.ion.org/plans/upload/PLANS20Program.pdf)
    
    - **SJTU University, Prof. Li Bijun**
        - [A Graph Approach to Dynamic Fusion of Sensors](https://www.ion.org/plans/upload/PLANS20Program.pdf), by Qianxun
    
    - **Collections**
        - [Deep urban unaided precise GNSS vehicle positioning](https://arxiv.org/pdf/1906.09539.pdf)
        - [Prediction of RTK-GNSS Performance in Urban Environments Using a 3D model and Continuous LoS Method](20200218/README.md)
    - **LAMBDA Algorithms and its variants**
        - [On the Best Integer Equivariant Estimator for Low-cost Single-frequency Multi-GNSS RTK Positioning](20200217/README.md)

- **GNSS SPP/PPP with FGO and Robust Model**

    - **Tim Pfeifer, Technique University of Chemniz, German**
        - [Incrementally learned Mixture Models for GNSS Localization](https://www.tu-chemnitz.de/etit/proaut/publications/pfeifer19_IV.pdf)
        - [Expectation-Maximization for Adaptive Mixture Models in Graph Optimization](https://www.tu-chemnitz.de/etit/proaut/publications/pfeifer19_ICRA.pdf)
        - [Robust Sensor Fusion with Self-tuning Mixture Models](https://www.tu-chemnitz.de/etit/proaut/publications/IROS2018.pdf)

    - **Ryan Watson, West Viginia University**
        - [Robust Incremental State Estimation through Covariance Adaptation](https://arxiv.org/pdf/1910.05382)
        - [Uncertainty Model Estimation in an Augmented Data Space for Robust State Estimation](https://arxiv.org/pdf/1908.04372)
        - [Enabling Robust State Estimation through Measurement Error Covariance Adaptation](https://arxiv.org/pdf/1906.04055)
        - [Batch Measurement Error Covariance Estimation for Robust Localization](https://www.researchgate.net/profile/Ryan_Watson7/publication/328643146_Batch_Measurement_Error_Covariance_Estimation_for_Robust_Localization/links/5bdb652ea6fdcc3a8db6d7ee/Batch-Measurement-Error-Covariance-Estimation-for-Robust-Localization.pdf)
        - [Evaluation of kinematic precise point positioning convergence with an incremental graph optimizer](https://arxiv.org/pdf/1804.04197)
        - [Robust Navigation In GNSS Degraded Environment Using Graph Optimization](https://arxiv.org/pdf/1806.08899)
        - [Flight Data Assessment of Tightly-Coupled PPP/INS using Real-Time Products](https://web.statler.wvu.edu/~gross/docs/IEEE_AESSM_TightPPP_INS_wRTGxGDGPS_R2.pdf)

### GNSS/INS Integration

- [LiDAR_GNSS_Mapping with GNSS/INS integration](https://github.com/ZhuangYanDLUT/lidar_gnss_mapping/blob/master/lidar_gnss_mapping/src/gps_ins_node.cpp)
- [GNSSINSLib](https://github.com/weisongwen/GNSSINSLib)
- [pose ekf with loosely coupled integration of GNSS/INS/Mag/Baro](https://github.com/libing64/pose_ekf)
- [GNSS INS integration using factor graph optimization from Tim, the INS calculation is based on SINS](https://github.com/weisongwen/ION_PLANS_2020)
- [KalmanFilter-Vehicle-GNSS-INS from Bonn University](https://github.com/alirezaahmadi/KalmanFilter-Vehicle-GNSS-INS)
- [Vision-based approach for GPS Localization Improvement in Semi-Obstructed Areas](https://github.com/alirezaahmadi/GNSS-AR)
- [Eagleye is an open-source software for vehicle localization utilizing GNSS and IMU](https://github.com/MapIV/eagleye)


### LiDAR SLAM
- **LOAM**
    - [LOAM: Lidar Odometry and Mapping in Real-time](http://www.roboticsproceedings.org/rss10/p07.pdf) 
        - first paper propose LOAM
    - [Tightly Coupled 3D Lidar Inertial Odometry and Mapping](https://arxiv.org/pdf/1904.06993.pdf) [[Code](https://github.com/hyye/lio-mapping)]
    - [LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain](https://arxiv.org/pdf/1904.06993.pdf) [[Code](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)]
    - [Loam_livox: A fast, robust, high-precision LiDAR odometry and mapping package for LiDARs of small FoV](https://arxiv.org/pdf/1909.06700.pdf) [[Code](https://github.com/hku-mars/loam_livox)]
    - [Optimized LOAM Using Ground Plane Constraints and SegMatch-Based Loop Detection](https://www.mdpi.com/1424-8220/19/24/5419/pdf) 
    - [Advanced implementation of LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) [[Code](https://github.com/HKUST-Aerial-Robotics/A-LOAM)]

- **LOAM+ multiple Sensors**
    - [Low-Cost GPS-Aided LiDAR State Estimation and Map Building](20200106/README.md): Day 06th, Jan 2020
    - [A Robust Laser-Inertial Odometry and Mapping Method for Large-Scale Highway Environments](https://www.researchgate.net/publication/338115486_A_Robust_Laser-Inertial_Odometry_and_Mapping_Method_for_Large-Scale_Highway_Environments) 
    - [Real-time, environmentally-robust 3d lidar localization](https://arxiv.org/pdf/1910.12728.pdf) 
    - [LINS: A Lidar-Inerital State Estimator for Robust and Fast Navigation](https://arxiv.org/pdf/1907.02233.pdf) 
    - [LiDAR and Inertial Fusion for Pose Estimation by Non-linear Optimization](https://arxiv.org/pdf/1710.07104.pdf)
    
    

- **NDT** 
    - [A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement](https://www.researchgate.net/publication/331283709_A_portable_three-dimensional_LIDAR-based_system_for_long-term_and_wide-area_people_behavior_measurement) [[Code](https://github.com/koide3/hdl_graph_slam)]
    - [3d lidar slam package based on NDT](https://github.com/ningwang1028/lidar_slam_3d) [[Code](https://github.com/ningwang1028/lidar_slam_3d)]

- **LiDAR Camera Inertial Fusion** 
    - [LIC-Fusion: LiDAR-Inertial-Camera Odometry](https://arxiv.org/pdf/1909.04102.pdf) 
    - [LIPS: LiDAR-Inertial 3D Plane SLAM](https://www.researchgate.net/profile/Yulin_Yang3/publication/327514859_LIPS_LiDAR-Inertial_3D_Plane_SLAM/links/5b92ebdd299bf1473923ca77/LIPS-LiDAR-Inertial-3D-Plane-SLAM.pdf)
    - [ndt_mapping_localization](https://github.com/melhousni/ndt_mapping_localization)
    ```
    C++ Implementation of the NDT mapping and localization algorithm for ADV on ROS.

    Two packages available in this implementation :

    vehicle_mapping : Pointcloud registration using the 3D NDT algorithm assisted by an EKF.
    vehicle_localization : 6-DoF Localization using the 3D NDT algorithm assisted by an EKF.
    ```
    [ndt_mapping and matching from Autoware](https://github.com/rsasaki0109/ndt_mapping)

### Visual/inertial SLAM

- [VISUAL-INERTIAL NAVIGATION: A CONCISE REVIEW](20200107/README.md)
- [GSLAM (A General SLAM Framework and BenchMark)](https://github.com/zdzhaoyong/GSLAM)
- [Visual SLAM Based on Dynamic Object Removal](https://ieeexplore.ieee.org/document/8961397)


### Sensor Fusion Framework
- [Tightly Coupled 3D Lidar Inertial Odometry and Mapping](https://arxiv.org/pdf/1904.06993.pdf) [[Code](https://github.com/hyye/lio-mapping)]
- [A Robust and Versatile Monocular Visual-Inertial State Estimator](https://arxiv.org/pdf/1708.03852.pdf) [[Code](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)]
-[GNSS/INS/LiDAR-SLAM Integrated Navigation System Based on Graph Optimization](https://www.mdpi.com/2072-4292/11/9/1009/pdf)

### Improve factor graph optimization
- [MASAT intial guess for SLAM](https://github.com/karoly-hars/MASAT_IG_for_SLAM) [[Code](https://github.com/karoly-hars/MASAT_IG_for_SLAM)]

### Autonomous Driving
- [Stanford Self Driving Car Code](https://github.com/emmjaykay/stanford_self_driving_car_code) [[Code](https://github.com/emmjaykay/stanford_self_driving_car_code)]

## Cooperative positioning
- [TBD](20200106/README.md)
- [Collaborative SLAM Patrik Schmuck from ETHZ](https://scholar.google.com/citations?hl=en&user=ssNhoMgAAAAJ&sortby=pubdate&view_op=list_works&citft=1&citft=2&email_for_op=wenwsrobo%40gmail.com&gmla=AJsN-F7zDqQ26Xyb5SOvvZMzQlZ7gEPwOjOvfCCHn44Y5UrwfQwD23SA6xWXS1w5GfQ_ma0FPkHUUJuvXAnCQGQgMW3F-JoprpaLQzpMIKPshIDrFa4IlQNsjHrDccubOyx506EtYcNrRwAyI_eIQMTdL3I3GMltZ4fTiZdan2FM0JJRk6vT7QVk9gH62s8aAKjetgffjc3504CLe3tytTTet85RS21RUKjygkZ9LGnezItmxqyoBgs8VUeMvUrQpbtZdK3Oi97d)


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)