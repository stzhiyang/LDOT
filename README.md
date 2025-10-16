# LV-DOT: LiDAR-Visual Dynamic Obstacle Detection and Tracking for Autonomous Robots
[![ROS1](https://img.shields.io/badge/ROS1-Noetic-blue.svg)](https://wiki.ros.org/noetic)
[![Linux platform](https://img.shields.io/badge/platform-Ubuntu-27AE60.svg)](https://releases.ubuntu.com/20.04/)
[![license](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) 
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Linux platform](https://img.shields.io/badge/platform-linux--arm-brown.svg)](https://releases.ubuntu.com/20.04/)


This repository implements the LiDAR-visual Dynamic Obstacle Detection and Tracking (LV-DOT) framework which aims at detecting and tracking dynamic obstacles for robots with extremely constraint computational resources.

<table>
  <tr>
    <td><img src="media/LV-DOT-demo1.gif" style="width: 100%;"></td>
    <td><img src="media/LV-DOT-demo2.gif" style="width: 100%;"></td>
  </tr>
  <tr>
    <td><img src="media/LV-DOT-demo3.gif" style="width: 100%;"></td>
    <td><img src="media/LV-DOT-demo4.gif" style="width: 100%;"></td>
  </tr>
</table>

The LV-DOT framework supports dynamic obstacle detection and tracking with multiple sensor configurations:
 - Camera-only mode.
 - LiDAR-only mode.
 - Combined LiDAR and camera mode.


For additional details, please refer to the related paper available here:


Zhefan Xu\*, Haoyu Shen\*, Xinming Han, Hanyu Jin, Kanlong Ye, and Kenji Shimada, "LV-DOT: LiDAR-visual dynamic obstacle detection and tracking for autonomous robot navigation”, arXiv, 2025. [\[preprint\]](https://arxiv.org/pdf/2502.20607) [\[YouTube\]](https://youtu.be/rRvgTulWqvk) [\[BiliBili\]](https://www.bilibili.com/video/BV1qC9GY6EHj/?share_source=copy_web&vd_source=1333db331406abb1b5d4cece1e253427)

*The authors contributed equally.


## News
- **2025-02-28:** The GitHub code, video demos, and relavant papers for our LV-DOT framework are released. The authors will actively maintain and update this repo!

## Table of Contents
- [Installation Guide](#I-Installation-Guide)
- [Run Demo](#II-Run-Demo)
    - [Run on dataset](#a-Run-on-dataset)
    - [Run on your device](#b-Run-on-your-device)
- [LV-DOT Framework and Results](#III-LV-DOT-Framework-and-Results)
- [Citation and Reference](#IV-Citation-and-Reference)
- [Acknowledgement](#V-Acknowledgement)


## I. Installation Guide
The system requirements for this repository are as follows. Please ensure your system meets these requirements:
- Ubuntu 18.04/20.04 LTS
- ROS Melodic/Noetic

This package has been tested on the following onboard computer:
- [NVIDIA Jetson Xavier NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-series/)
- [NVIDIA Jetson Orin NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) 
- [Intel NUC](https://www.intel.com/content/www/us/en/products/details/nuc.html)
 

Please follow the instructions below to install this package.
```
# This package needs ROS vision_msgs package
sudo apt install ros-noetic-vision-msgs

# Install YOLOv11 required package
pip install ultralytics

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/LV-DOT.git
cd ..
catkin_make
```


## II. Run Demo
### a. Run on dataset
Please download the rosbag file from this [link](https://cmu.box.com/s/cucvje5b9xfpdpe57ilh0jx702b3ks2p):
```
rosbag play -l corridor_demo.bag
roslaunch onboard_detector run_detector.launch
```
The perception results can be visualized in Rviz as follows:

https://github.com/user-attachments/assets/e640edab-d4f3-40d6-88dc-9e5014430732


### b. Run on your device
Please adjust the configuration file under ```cfg/detector_param.yaml``` of your LiDAR and camera device. Also, change the color image topic name in ```scripts/yolo_detector/yolov11_detector.py```

From the parameter file, you can find that the algorithm expects the following data from the robot:
- LiDAR Point Cloud: ```/pointcloud```

- Depth image: ```/camera/depth/image_rect_raw```

- Color image: ```/camera/color/image_rect_raw```

- Robot pose: ```/mavros/local_position/pose```

- Robot odometry (alternative to robot pose): ```/mavros/local_position/odom```

Additionally, update the camera intrinsic parameters and the camera-LiDAR extrinsic parameters in the config file.

Run the following command to launch dynamic obstacle detection and tracking.
```
# Launch your sensor device first. Make sure it has the above data.
roslaunch onboard_detector run_detector.launch
```

The LV-DOT can be directly utilized to assist mobile robot navigation and collision avoidance in dynamic environments, as demonstrated below:

<table>
  <tr>
    <td><img src="media/navigation-demo.gif" style="width: 100%;"></td>
    <td><img src="media/avoidance-demo.gif" style="width: 100%;"></td>
  </tr>
</table>

## III. LV-DOT Framework and Results
The LV-DOT framework is shown below. Using onboard LiDAR, camera, and odometry inputs, the LiDAR and depth detection modules detect 3D obstacles, while the color detection module identifies 2D dynamic obstacles. The LiDAR-visual fusion module refines these detections, and the tracking module classifies obstacles as static or dynamic.

<p align="center">
  <img src="https://github.com/user-attachments/assets/5352c7ae-341a-45c0-8ee1-253d9aed6078" width="90%">
</p>

Example qualitative perception results in various testing environments are shown below:
<p align="center">
  <img src="https://github.com/user-attachments/assets/054e3285-4c44-49e3-939d-74176c5d676e" width="90%">
</p>


## IV. Citation and Reference
If our work is useful to your research, please consider citing our paper.
```
@article{LV-DOT,
  title={LV-DOT: LiDAR-visual dynamic obstacle detection and tracking for autonomous robot navigation},
  author={Xu, Zhefan and Shen, Haoyu and Han, Xinming and Jin, Hanyu and Ye, Kanlong and Shimada, Kenji},
  journal={arXiv preprint arXiv:2502.20607},
  year={2025}
}
```
## V. Acknowledgement
The authors would like to express their sincere gratitude to Professor Kenji Shimada for his great support and all CERLAB UAV team members who contribute to the development of this research.

