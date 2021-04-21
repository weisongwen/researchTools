#!/usr/bin/env bash 
# gnome-terminal -x bash -c "./pointcloud_to_laserscan_processing.sh;exec bash;"
# gnome-terminal -x bash -c "./skyplot_visualization.sh;exec bash;"
# gnome-terminal -x bash -c "./openbag.sh;exec bash;"
gnome-terminal -x bash -c "./laserOdometry.sh;exec bash;"
gnome-terminal -x bash -c "./optimization.sh;exec bash;"

