# Notes for install the driver for point grey camera and Xsense MTi 30 driver for data collection
install the driver for point grey camera and Xsense MTi 30 driver for data collection

### Reference
1. [(Install Camera SDK)](https://www.cnblogs.com/zhchp-blog/p/8040247.html)
2. [Install camera ros driver](https://github.com/ros-drivers/pointgrey_camera_driver)
3. [how to run ros driver](http://wiki.ros.org/pointgrey_camera_driver)

```
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
roslaunch pointgrey_camera_driver camera.launch
roslaunch xsens_driver xsens_driver.launch 
```
### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)