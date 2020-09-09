# calibration_IMUCamera

## How to get camera intrinsic parameters?
-   There are two ways: 
1. By ROS, you can follow below tutorials:
    http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
2. By the toolbox: kalibr
    https://github.com/ethz-asl/kalibr

   Actually, when i use the toolbox, i met some problems. so i changed the original command to the following, the problems were solved:
   
   PS.  Building from source, had better not directly use smartphone bagfile as this tool is not applicable for smartphone.
   ```
   kalibr_calibrate_cameras --target data/april_6x6.yaml --bag data/2020-09-06-16-26-30.bag --bag-from-to 10 50 --models pinhole-radtan --topics /camera/image_color --show-extraction
   ```
   Notes (important) :  how to read the report?

## How to get IMU parameters?
 - By imu_utils, this package by gaowenliang(HKUST)
 https://github.com/gaowenliang/imu_utils

   The tutorial is not very clear, so i reference some blog: 
 https://blog.csdn.net/fang794735225/article/details/92804030

1. after downloading two packages: imu_utils, code_utils, then firstly put code_utils under src to compile, if successfull, then put imu_utils under same directory to compile;
1. next, record imu data for 2 hours;
2. calibrate imu by two steps(i choose xsens);
   
   rosbag play -r 200　filename.bag 

   roslaunch imu_utils x.launch


## How to calibrate IMU-Camera extrinsic parameters?
kalibr_calibrate_imu_camera --target data/april_6x6.yaml --cam cam.yaml --imu xsens_imu.yaml --bag data/2020-09-06-16-26-30.bag --bag-from-to 10 50 --show-extraction
- For reference: https://blog.csdn.net/u011178262/article/details/83316968
# My small sensor kit parameters:

<p align="center">
  <img width="300pix" src="smallSensorKit.jpg">
</p>


```
Camera intrinsic (by ros):
model_type: PINHOLE
camera_name: camera
image_width: 1920
image_height: 1200
camera intrinsic:
distortion_parameters:
   k1: -0.109203
   k2:  0.063536
   p1: -0.003427
   p2: -0.000629
projection_parameters:
   fx: 1086.160899
   fy: 1090.242963
   cx: 940.067502
   cy: 586.740077
```
```
IMU intrinsic(the following format needed change when use kalibr):
%YAML:1.0
---
type: IMU
name: xsens
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 7.4476180445963098e-03
      gyr_w: 8.2775665766972293e-05
   x-axis:
      gyr_n: 7.2504888854417662e-03
      gyr_w: 8.7356692540001794e-05
   y-axis:
      gyr_n: 7.8258358855956486e-03
      gyr_w: 7.3688872009799617e-05
   z-axis:
      gyr_n: 7.2665293627515145e-03
      gyr_w: 8.7281432751115480e-05
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 8.1751044906501914e-03
      acc_w: 1.3084246540207204e-04
   x-axis:
      acc_n: 9.0886759513343560e-03
      acc_w: 1.3705658898071130e-04
   y-axis:
      acc_n: 8.2607696663920521e-03
      acc_w: 1.0905439458637021e-04
   z-axis:
      acc_n: 7.1758678542241635e-03
      acc_w: 1.4641641263913455e-04
```
```
extrinsic parameter between camera and imu:
```
```
cam0 (transformation from IMU to camera coordinates):
  T_cam_imu:
  - [-0.9980910026236224, -0.0015799642414101797, -0.06174021537674852, 0.054912613927649394]
  - [-0.06172758897811955, -0.007075633703491782, 0.998067953681833, 0.11622983489906714]
  - [-0.002013762826095295, 0.9999737192150048, 0.006964598954021748, -0.11470539005102824]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [-0.109203, 0.063536, -0.003427, -0.000629]
  distortion_model: radtan
  intrinsics: [1086.160899, 1090.242963, 940.067502, 586.740077]
  resolution: [1920, 1200]
  rostopic: /camera/image_color
  timeshift_cam_imu: -0.02110350429911481
```
Interestingly, I found that had better directly put xsens_imu.yaml, cam.yaml into src/ instead of data/, but you can put bag data and target file into data/, I do not why but it really works.



### Contact
- Author: xiwei, PhD Candidate in Hong Kong Polytechnic University.
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)