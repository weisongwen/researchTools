# C++ std::vector operation

## Common operation
- **initialization of vector**

    
- reduce vector based on the other vector
```
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
```

- initialize std::map using emplace_back()
```C++
/* prepare the output of feature tracking */
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
for (size_t i = 0; i < ids.size(); i++)
{
    int feature_id = ids[i];
    double x, y ,z;
    x = cur_un_pts[i].x;
    y = cur_un_pts[i].y;
    z = 1;
    double p_u, p_v;
    p_u = cur_pts[i].x;
    p_v = cur_pts[i].y;
    int camera_id = 0;
    double velocity_x, velocity_y;
    velocity_x = pts_velocity[i].x;
    velocity_y = pts_velocity[i].y;

    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
}
```
- initialization of std::map using insert 
```C++
vector<cv::Point2f> pts_velocity;
cur_id_pts.clear();
for (unsigned int i = 0; i < ids.size(); i++)
{
    /* construct the cur_id_pts based on pts and ids */
    cur_id_pts.insert(make_pair(ids[i], pts[i]));
}
```
### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)