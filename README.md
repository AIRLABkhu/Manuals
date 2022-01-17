# 2022 AIR LAB Winter Research : Pick and Place

이것은 경희대학교 AIR Lab (Artificial Intelligence and Robotics) 소속 [박지원 학부연구생](https://zzziito.github.io/)이 진행하고 있는 프로젝트입니다. <br/><br/>
*This is a project led by Jiwon Park (Undergraduate Student of AIR LAB, Kyung Hee Univ).*

## Table of Contents

1. realsense 를 이용한 3D Visualization


## 프로젝트 개요 

로봇팔의 원활한 **Pick and Place** 동작 수행을 위해서, <br/>
Perception 부터 Manipulation 까지 직접 개발하는 것을 목표로 하고 있습니다. <br/><br/>
*For the smooth Pick and place operation of the robot arm, we want to develop from Perception to Manipulation.*

### Device

+ **Intel Realsense** Depth Camera D435 : RGB-D 카메라

### Software

+ **PYTHON** with Opencv, Pyrealsense2

### OS

+ Ubuntu 18.04

### IDE

+ Vscode 

### Week 1. OpenCV Application Using Realsense. 

realsense 를 사용하여 카메라 화면을 image, depth, 3D point cloud 로 변환하여 보여주는 Application 을 만드는 것이 1주차 목표입니다. 

#### Install Pyrealsense2 in your Ubuntu desktop

[Intel Realsense Github](https://github.com/IntelRealSense/librealsense)

pyrealsense2 패키지는 Realsense 카메라를 Python 으로 제어할 수 있는 패키지입니다. <br/>
본 프로젝트에서는 직접 데이터를 다루기 위해 SDK 가 아닌, wrapper 를 사용합니다. 

'pip install pyrealsense2' command 는 Ubuntu, macOS 에서는 제공되지 않습니다. <br/>
macOS 에서 사용하는 것보다, Ubuntu 에서 사용하는 것이 더 쉬울 수 있습니다. 

Ubuntu 에 설치를 위해서는 [링크](https://lieuzhenghong.com/how_to_install_librealsense_on_the_jetson_nx/) 를 참고해 주세요. 

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

이후 작성한 코드는 python3 로 실행해 주세요. 

#### Application Making

realsense github 코드를 참고하여 작성했습니다.

![week1](https://user-images.githubusercontent.com/52185595/148065851-07799b4b-dcc8-486b-b7f2-98a89b9dde3c.gif)

[week1 실행 코드](https://github.com/AIRLABkhu/Pick_and_Place/blob/main/week1.py)

**기능**

keyboard input

+ w : 3D viewer 
+ e : 2D / depth viewer
+ p : pause
+ r : reset view
+ z : toggle point scaling
+ c : toggle color source
+ s : save png

**Project 함수**

Pyrealsense 에서는 3D Point Cloud 를 표현하기 위해 Open3D 등의 라이브러리를 사용하는 대신, project 라는 함수를 사용합니다. <br/>
이 함수는 numpy 를 이용해 3d vector array 를 2d 로 표현합니다. <br/>

**Painter's Algorithm**

Point Cloud 함수에서는 point cloud 를 painter's algorithm 에 근거하여 표시합니다. 
painter's algorithm 은 먼 곳의 point 부터 먼저 표시하는 것을 의미합니다. 



### Week 2. Open3d Application Using Kinect. 

Azure Kinect 와 Open3d 를 사용하여 카메라 화면을 image, depth, 3D point cloud 로 변환하여 보여주는 Application 을 만드는 것이 1주차 목표입니다. 

![kinect](https://user-images.githubusercontent.com/52185595/149642458-b52f2c13-9610-412f-919c-ab719007ab5b.png)

depth camera streaming

![ezgif com-gif-maker](https://user-images.githubusercontent.com/52185595/149646546-f291d3a3-3f6b-4513-9dd3-ff80a5205ac0.gif)

![ezgif com-gif-maker (2)](https://user-images.githubusercontent.com/52185595/149692164-698d9a87-8436-4873-836a-a17a65e58b03.gif)


o3d.geometry.PointCloud.create_from_rgbd_image() 와 <br/>
o3d.visualization.draw_geometries() 를 이용하여 생성한 point cloud


![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/52185595/149689730-f7b71ef8-cc50-45a4-9caa-b037df75b19d.gif)




#### Ubuntu 에서 Azure Kinect 사용 환경 세팅하기 

[Install Azure Kinect SDK on Ubuntu](https://tianyusong.com/2019/11/13/how-to-install-azure-kinect-sdk-on-ubuntu-16-04/)

먼저 SDK 를 먼저 설치합니다. 

이 과정에서 생기는 에러를 해결하는 방법은 하단의 Troubleshooting 부분을 참고해 주세요. 
같은 폴더 내에 default_config.json 파일이 있어야 한다는 점을 유의해 주세요. 





### Troubleshooting

**Pipeline 오류**

```
AttributeError: module ‘pyrealsense2’ has no attribute ‘pipeline’
```


다음과 같은 에러 메시지가 뜰 때는 \__init__\.py 파일을 site_packages/pyrealsense2 폴더 내에 위치시키면 해결할 수 있습니다. 
[참고](https://github.com/IntelRealSense/librealsense/issues/6820)


#### Azure Kinect 를 사용할 때 맞닥뜨리게 되는 여러가지 에러들


Azure Kinect 를 Ubuntu 환경에서 사용하도록 세팅하는 작업은 복잡하고 번거로워서 여러 날이 걸릴 수 있습니다. 차근차근 에러를 해결해 나가 보아요. 

<img width="480" alt="스크린샷 2022-01-16 오전 10 32 25" src="https://user-images.githubusercontent.com/52185595/149643891-82ddd9da-eb51-4a26-9566-4f95979c76e7.png">

**a. Cannot Connect to device**
<br/>
일단 depth camera 를 disable 시키고 color camera 만 enable 시켜서 열어보세요. 
열리나요? 


그렇다면 이 문제는 컴퓨터에 depth camera 를 위한 드라이버가 설치되어 있지 않은 것이 원인일 수 있습니다. 
Azure Kinect 는 depth camera 를 사용하기 위해서 
```
k4a-tools
libk4a
libk4a-dev
```
다음 세 가지에 대한 설치를 요구합니다. 

[Depth Camera Connot Be Loaded](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1453)

이 라이브러리들의 버전을 동일하게 맞추는 것이 필수적입니다. 이미 설치했다면, 

```
apt list --installed | grep k4a 
```
로 버전을 확인해보세요. 버전이 동일한가요?

![kinect_library](https://user-images.githubusercontent.com/52185595/149643785-bcd3bef1-cfc8-4853-b6bf-3f305365451a.png)


**b. OpenGL Version 문제**
<br/>
```
OpenGL 4.4 not supported. Please install latest graphics drive. 
```

[OpenGL Upgrade 하는 방법](https://www.reddit.com/r/Ubuntu/comments/8tpq05/how_can_update_my_display_driver_to_opengl_33/)

<img width="693" alt="스크린샷 2022-01-16 오전 11 19 09" src="https://user-images.githubusercontent.com/52185595/149644790-9bc1b6d7-acff-44bb-90d0-61ccb79b6f19.png">

Azure Kinect 를 사용하려면 최소한 OpenGL 4.4 이상의 버전이 필요합니다. 버전을 확인하고 4.4 미만의 버전이라면 위의 사이트를 참고해서 OpenGL 을 업그레이드하되, 저도 늘 되는 방법은 아니었습니다. 
더 좋은 방법이 있다면 알려주세요. 

**c. pip install open3d - cannot import open3d 문제**
<br/>
python 3.9 환경에서 pip install open3d 를 실행했는데도 import open3d 를 했을 때 

```
no module named "Open3d" 
```
라는 에러가 떴다면, 3.9 미만의 버전으로 python virtualenv 를 만드는 방법을 고려해보세요. 

**d. framerate 문제**
<br/>
아직도 depth camera 가 SDK 에서 열리지 않았나요? 
framerate 를 30 fps 에서 5 fps 로 바꿔 보실래요? 

**e. sudo 문제**
<br/>

```
On Ubuntu, you’ll need to set up udev rule to use the Kinect Camera without sudo. 
```
라는 문구를 주의 깊게 읽으세요! 

Kinect 를 연결하고 Kinect SDK 도 정상적으로 열리는데도 python3 test1.py 를 실행했을 때, 

```
libusb devices are all unavailable
k4a_plugin::k4a_device_open() failed
runtime error: failed to connect to sensor
```

다음과 같은 에러가 뜬다면, 그것은 관리자 권한으로 실행하지 않아 kinect 가 열리지 않기 때문일 수 있습니다. 

[Use Kinect without sudo](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup)

이때 

```
$ sudo python3 test1.py
```

를 했을 때 no module named... 같은 module import error 가 뜨는 이유는  sudo python3 와 python3 가 참조하는 루트가 다르기 때문입니다.

**e. usb 포트 문제**
<br/>
특정 usb 포트에 꽂았을 때만 kinect 가 실행되는 경우가 있었습니다. 조합을 바꿔서 여러 군데 꽂아보세요. 
