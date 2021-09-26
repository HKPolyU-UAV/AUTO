# AUTO (an "A"utonomous "U"av that "T"racks "O"bject)
##  An Autonomous Object Tracking UAV System for Surveillance Applications
This project is based upon the state-of-the-art [YOLO series, Tiny](https://github.com/AlexeyAB/darknet#how-to-train-tiny-yolo-to-detect-your-custom-objects) and the famous Kalman Filter. With both camera and object moving, our system is able to track the target robustly in a 3D world. The UAV would then try to maneuver with the dynamic object once it detects the movement from the target.

### Abstract
<div align="justify">
The ever-burgeoning growth of autonomous unmanned aerial vehicles (UAVs) has demonstrated a promising platform for utilization in real-world applications. In particular, UAV equipped with vision system could be leveraged for surveillance applications. This paper proposes a learning-based UAV system for achieving autonomous surveillance, in which the UAV can be of assistance in autonomously detecting, tracking, and following a target object without human intervention. Specifically, we adopted the deep learning-based YOLOv4-Tiny algorithm for semantic object detection and then further consolidated it with a 3D object pose estimation method and Kalman Filter to enhance the perception performance. In addition, a back-end UAV path planning for surveiilance maneuver is embedded to complete the fully autonomous system. The perception module is assessed on a quadrotor UAV, while the presented system is validated through flight experiments in the VICON motion-capture environments. The experiments result verifies the robustness, effectiveness, and reliability of the autonomous object tracking UAV system in performing surveillance tasks. The source code is released to the community for future research reference.
</div>


### Video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=tY16YnZQoB4
" target="_blank"><img src="http://img.youtube.com/vi/tY16YnZQoB4/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="533" height="400" border="10" /></a>


### Requirements
* We have validated our system on **Ubuntu 18.04** [ubuntu release](https://releases.ubuntu.com/)
* Installation of ROS: **ROS or Melodic:** [ROS Install](http://wiki.ros.org/ROS/Installation)
* Requires **OpenCV >= 4.4:** [OpenCV Linux Install](https://docs.opencv.org/4.4.0/d7/d9f/tutorial_linux_install.html)
* **Python 3.8** 
* **CUDNN >= 7.0:** [cuDNN Archive](https://developer.nvidia.com/rdp/cudnn-archive)
* Realsense libraries [librealsense github](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
* Realsense ros-wrapper, we suggest users to build from source [ros wapper](https://github.com/IntelRealSense/realsense-ros#step-2-install-intel-realsense-ros-from-sources)

### Dataset Establishment and Training
* **YOLO Darknet** here: [Darknet](https://github.com/pjreddie/darknet)
* Suggested dataset scale: 2000 images per class && corresponding 500 for validation, and 2000 background images without object in the FoV
* Labelling tool: [labelimg](https://tzutalin.github.io/labelImg/) && [labelimg repo](https://github.com/tzutalin/labelImg)

### Clone our work! (on ubuntu)
1. clone our repository into working space

```
cd ~/xx_ws/src
git clone https://github.com/pattylo/Autonomous-Object-Tracking-UAV-System.git
```

2. Compile 
```
cd ~/xx_ws
catkin_make
```

3. Run
```
rosrun offb camera && rosrun offb track
#or can just write a launch file
```


## Maintainer 
**Patrick Li-yu LO**: liyu.lo@connect.polyu.hk<br/>
Repo Co-Contributor:<br/> **Summer Chi Hao Yiu** chi-hao.yiu@connect.polyu.hk && <br/> **Bryant Yu Tang** bryant.tang@connect.polyu.hk
