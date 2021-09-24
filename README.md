# AUTO (an **A**utonomous **U**av that **T**racks **O**bject)
##  An Autonomous Object Tracking UAV System for Surveillance Applications
This project is based upon the state-of-the-art [YOLO series](https://github.com/AlexeyAB/darknet) and the famous Kalman Filter. With both camera and object moving, our system is able to track the target robustly in a 3D world.


### Video

### Requirements
* We have validated our system on **Ubuntu 18.04**
* Installation of ROS: **ROS or Melodic:** [ROS Install](http://wiki.ros.org/ROS/Installation)
* Requires **OpenCV >= 4.4:** [OpenCV Linux Install](https://docs.opencv.org/4.4.0/d7/d9f/tutorial_linux_install.html)
* **Python 3.8** 
* & **CUDNN >= 7.0:** [cuDNN Archive](https://developer.nvidia.com/rdp/cudnn-archive)
* Realsense libraries [librealsense github](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
* Realsense ros-wrapper, we suggest users to build from source [ros wapper](https://github.com/IntelRealSense/realsense-ros#step-2-install-intel-realsense-ros-from-sources)

### Dataset Establishment and Training
* **YOLO Darknet** here: [Darknet](https://github.com/pjreddie/darknet)
* Suggested dataset scale: 2000 images per class && corresponding 500 for validation 
* Labelling tool: [labelimg]https://tzutalin.github.io/labelImg/ [labelimg repo](https://github.com/tzutalin/labelImg)

### Environment Installation (on ubuntu)
1. clone our repository into working space

```
cd ~/xx_ws/src
git clone https://github.com/pattylo/Autonomous-Object-Tracking-UAV-System
```

2. Compile 
```
cd ~/xx_ws
catkin_make
```


## Maintainer 
Patrick Li-yu LO (Dept.AAE,PolyU): [liyu.lo@connect.polyu.hk](liyu.lo@connect.polyu.hk)
Contributor: Summer Chi Hao Yiu && Bryant Yu Tang
