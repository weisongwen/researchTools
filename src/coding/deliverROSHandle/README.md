# deliver ros handle from main node cpp to a library cpp
sometimes, we want to publish the middle results from the cpp file which belongs to a specific package. For example the RTKLIB, we want to publish the GNSS data from file pntpos.cpp in RTKLIB, we can use the following function:
```
extern void pntposRegisterPub(ros::NodeHandle &n)
{
    pub_pntpos_odometry = n.advertise<nav_msgs::Odometry>("pntpos_odometry", 1000);
    pub_gnss_raw = n.advertise<nlosExclusion::GNSS_Raw_Array>("GNSS_RTKLIB", 1000);

    pub_wls_odometry = n.advertise<nav_msgs::Odometry>("wlspose_odometry", 1000);

    pub_velocity_from_doppler = n.advertise<nav_msgs::Odometry>("velocity_from_doppler", 1000);


}
```

be noted to add the following code at the node cpp
```
/*------------------------------------------------------------------------------
* rnx2rtkp.c : read rinex obs/nav files and compute receiver positions
*
*          Copyright (C) 2007-2009 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
* history : 2007/01/16  1.0 new
*           2007/03/15  1.1 add library mode
*           2007/05/08  1.2 separate from postpos.c
*           2009/01/20  1.3 support rtklib 2.2.0 api
*           2009/12/12  1.4 support glonass
*                           add option -h, -a, -l, -x
*           2010/01/28  1.5 add option -k
*           2010/08/12  1.6 add option -y implementation (2.4.0_p1)
*-----------------------------------------------------------------------------*/
/* RTKLIB Library */

#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"
#include <ros/ros.h>
#include <stdio.h>
#include <assert.h>
#define ENACMP 1

extern void postposRegisterPub(ros::NodeHandle &n);
extern void rtkposRegisterPub(ros::NodeHandle &n);
extern void pntposRegisterPub(ros::NodeHandle &n);

/* read .obs, .nav files and perform RTK GNSS and save results to file */

// #include "utilities.h"
// #include "WeightLeastSquare.h"
// #include "SelectEphemeris.h"

/* rnx2rtkp main -------------------------------------------------------------*/

int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "rtk_estimator");
	ros::NodeHandle nh("~");
    std::cout<<"......rtk_estimator node start......"<<std::endl;

    int n=0,i,stat;
	postposRegisterPub(nh);
	rtkposRegisterPub(nh);
	pntposRegisterPub(nh);
```
This application can also be found in the VINSFusion cpp file [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L34)

### Reference
1. [Visualization.cpp in VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/utility/visualization.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), PhD Candidate in Hong Kong Polytechnic University.
- Email: weisongwen@weisongwen
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)