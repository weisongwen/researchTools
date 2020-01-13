# Low-Cost GPS-Aided LiDAR State Estimation and Map Building([pdf](https://arxiv.org/pdf/1910.12731.pdf))

## Abstract
Using different sensors in an autonomous vehicle (AV) can provide multiple constraints to optimize AV location estimation. In this paper, we present a low-cost GPSassisted LiDAR state estimation system for AVs. Firstly, we utilize LiDAR to obtain highly precise 3D geometry data. Next, we use an inertial measurement unit (IMU) to correct point cloud misalignment caused by incorrect place recognition. The estimated LiDAR odometry and IMU measurement are then jointly optimized. We use a lost-cost GPS instead of a realtime kinematic (RTK) module to refine the estimated LiDARinertial odometry. Our low-cost GPS and LiDAR complement each other, and can provide highly accurate vehicle location information. Moreover, a low-cost GPS is much cheaper than an RTK module, which reduces the overall AV sensor cost. Our experimental results demonstrate that our proposed GPS-aided LiDAR-inertial odometry system performs very accurately. The accuracy achieved when processing a dataset collected in an industrial zone is approximately 0.14 m. 

**Authors**
Linwei Zheng1,2, Yilong Zhu1,2, Bohuan Xue1,2, Ming Liu1, Rui Fan2,3
1Shenzhen Unity-Drive Innovation Technology Co. Ltd., Shenzhen, China.
2Robotics Institute, Hong Kong University of Science and Technology, Hong Kong SAR, China.
3Hangzhou ATG Intelligent Technology Co. Ltd., Hangzhou, China.
Emails: zhenglinwei@unity-drive.com, {yzhubr, bxueaa, eelium, eeruifan}@ust.h

<p align="center">
  <img width="712pix" src="framework.png">
</p>

<p align="center">
  <img width="712pix" src="result.png">
</p>

## comments
LiDAR is firstly integrated with the INS pre-integration. Then the fused result is fused with the GNSS positioning.

**Advantage**: the LiDAR/INS result is smooth, then loosely integrate with the GNSS.
**Question**: 
- how to cope with the uncertainty in GNSS positioning?
- in fact, it is pretty hard to verify that the error is less than 0.2 meters in urban simply based on RTK GNSS. The accuracy is about 0.14 meters?