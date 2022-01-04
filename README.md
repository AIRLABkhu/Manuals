# 2022 AIR LAB Winter Research : Pick and Place

이것은 경희대학교 AIR Lab (Artificial Intelligence and Robotics) 소속 [박지원 학부연구생](https://zzziito.github.io/)이 진행하고 있는 프로젝트입니다. <br/><br/>
*This is a project led by Jiwon Park (Undergraduate Student of AIR LAB, Kyung Hee Univ).*

## Table of Contents

1. realsense 를 이용한 3D Visualization


## 프로젝트 개요 

로봇팔의 원활한 **Pick and Place** 동작 수행을 위해서, Perception 부터 Manipulation 까지 직접 개발하는 것을 목표로 하고 있습니다. <br/><br/>
*For the smooth Pick and place operation of the robot arm, we want to develop from Perception to Manipulation.*

#### Device

+ **Intel Realsense** Depth Camera D435 : RGB-D 카메라

#### Software

+ **PYTHON** with Opencv, Pyrealsense2

### OS

+ Ubuntu 18.04

### IDE

+ Vscode 

### Week 1. OpenCV Application Using Realsense. 

realsense 를 사용하여 카메라 화면을 image, depth, 3D point cloud 로 변환하여 보여주는 Application 을 만드는 것이 1주차 목표입니다. 

#### Install Pyrealsense2 in your Ubuntu desktop

pyrealsense2 패키지는 
'pip install pyrealsense2' command 는 Ubuntu, macOS 에서는 제공되지 않습니다. 

[링크](https://lieuzhenghong.com/how_to_install_librealsense_on_the_jetson_nx/) 를 참고하세요. 

```
$ sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
$ sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd ./librealsense
$ ./scripts/setup_udev_rules.sh
$ mkdir build && cd build
$ cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true
$ sudo make uninstall && sudo make clean && sudo make -j4 && sudo make install
$ export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2
```


