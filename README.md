# AUTO (an "A"utonomous "U"av that "T"racks "O"bject)

##  Dynamic Object Tracking on Autonomous UAV System for Surveillance Applications

<div align="justify">
This project is based upon the state-of-the-art <a href="https://github.com/AlexeyAB/darknet#how-to-train-tiny-yolo-to-detect-your-custom-objects">YOLO series, Tiny</a> and the famous Kalman Filter. With both camera and object moving, our system is able to track the target robustly in a 3D world. The UAV would then try to maneuver with the dynamic object once it detects the movement from the target. The link to this journal paper could be seen <a href="https://www.mdpi.com/1424-8220/21/23/7888">here</a>.
</div>



### Abstract
<div align="justify">
The ever-burgeoning growth of autonomous unmanned aerial vehicles (UAVs) has demonstrated a promising platform for utilization in real-world applications. In particular, UAV equipped with a vision system could be leveraged for surveillance applications. This paper proposes a learning-based UAV system for achieving autonomous surveillance, in which the UAV can be of assistance in autonomously detecting, tracking, and following a target object without human intervention. Specifically, we adopted the YOLOv4-Tiny algorithm for semantic object detection and then consolidated it with a 3D object pose estimation method and Kalman Filter to enhance the perception performance. In addition, a back-end UAV path planning for surveillance maneuver is integrated to complete the fully autonomous system. The perception module is assessed on a quadrotor UAV, while the whole system is validated through flight experiments. The experiment results verified the robustness, effectiveness, and reliability of the autonomous object tracking UAV system in performing surveillance tasks. The source code is released to the research community for future reference.
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
git clone https://github.com/pattylo/AUTO.git
```

2. Modify </br>
Go to [here](https://github.com/pattylo/AUTO/blob/master/offb/src/include/run_yolo.cpp#L19) if Cuda Available
```
//uncomment the below if CUDA available
    //this->mydnn.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    //this->mydnn.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;and [here](https://github.com/pattylo/AUTO/blob/master/offb/src/camera.cpp#L19)
```
//change yolo custom weight file location, as well as the cfg file and name file
```

3. Compile 
```
cd ~/xx_ws
catkin_make
```

4. Run
```
rosrun offb camera && rosrun offb track
#or can just write a launch file
```
## Cite Us
```
@article{lo2021dynamic,
  title={Dynamic Object Tracking on Autonomous UAV System for Surveillance Applications},
  author={Lo, Li-Yu and Yiu, Chi Hao and Tang, Yu and Yang, An-Shik and Li, Boyang and Wen, Chih-Yung},
  journal={Sensors},
  volume={21},
  number={23},
  pages={7888},
  year={2021},
  publisher={MDPI}
}
```

## Maintainer 
```
Patrick Li-yu LO: liyu.lo@connect.polyu.hk
Summer Chi Hao Yiu: chi-hao.yiu@connect.polyu.hk 
Bryant Yu Tang: bryant.tang@connect.polyu.hk
```
