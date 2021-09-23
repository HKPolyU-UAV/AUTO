# An Autonomous Object Tracking UAV System for Surveillance Applications
## Learning-based Autonomous Inspection UAV System
This project is based upon the state-of-the-art [YOLO series](https://github.com/AlexeyAB/darknet) and the famous Kalman Filter. With both camera and object moving, our system is able to track the target robustly in a 3D world.


### Video

### Requirements
* **Ubuntu 16.04 or 18.04**
* **ROS Kinetic or Melodic:** [ROS Install](http://wiki.ros.org/ROS/Installation)
* **OpenCV >= 4.4:** [OpenCV Linux Install](https://docs.opencv.org/4.4.0/d7/d9f/tutorial_linux_install.html)
* **Python 3.8** 
* **CUDA >= 10.0:** [CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) 
* **CUDNN >= 7.0:** [cuDNN Archive](https://developer.nvidia.com/rdp/cudnn-archive)

### Environment Installation (ubuntu)
1. clone repository into working space

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
Patrick LO(Dept.AAE,PolyU): [liyu.lo@connect.polyu.hk](liyu.lo@connect.polyu.hk)
