# C++ ROSBAG operations, commands 

## Common operation
- rosbag play with different topic name
    ```
    rosbag play --pause 2019-04-28-20-58-02.bag camera/image_color:=/camera/image_raw /imu/data:=/imu
    ```
    - the rostopic in the rosbag is  originally **camera/image_color**
    - modified topic is **camera/image_raw**
    - **--pause** means pause at the very begining
- loop play **rosbag play -l**
-  specific topics to play
```
rosbag play recorded1.bag --topics /topic1 /topic2 /topic3
``` 
### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)