# Usage of eigen for estimator, rotation, matrix

## C++ Array to eigen matrix 
    
```C++
// Covariance.
Eigen::Matrix<double, 15, 15> cov;
...
gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());
```

## absolute square, max values in eigen matrix 

```C++
/* Compute mean and std of the imu buffer*/
Eigen::Vector3d sum_acc(0., 0., 0.);
for (const auto imu_data : imu_buffer_) {
    sum_acc += imu_data->acc;
}
const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

Eigen::Vector3d sum_err2(0., 0., 0.);
for (const auto imu_data : imu_buffer_) {
    sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
}
const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

if (std_acc.maxCoeff() > kAccStdLimit) {
    LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
    return false;
}
```

### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)