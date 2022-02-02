# Velodyne VLP-16 3d Lidar

### Week 3. Velodyne Test

Velodyen VLP16 을 테스트해보고, 다양한 활용 방안을 생각해 보는 것이 3주차 목표입니다. 

**Setting**

<img src = "https://user-images.githubusercontent.com/52185595/150662760-bd5b3a03-ff5e-48b1-8d82-5309517dacf1.jpg" width="600" >

Velodyne Puck VLP16 을 구동하려면 패키지 구성상품 말고 추가로 110V 변압기 (돼지코) 와 ethernet 젠더가 필요합니다. 관련 내용은 
[벨로다인 세팅](https://www.youtube.com/watch?v=Pa-q5elS_nE&ab_channel=VelodyneLidar) 을 참고하세요. 

#### Veloview

![ezgif com-gif-maker (5)](https://user-images.githubusercontent.com/52185595/150662498-e5e1a72e-c9aa-4938-98e2-13fbab0eb9c1.gif)

#### ROS Rviz

![ezgif com-gif-maker (3)](https://user-images.githubusercontent.com/52185595/150643757-9e4d0a66-6d75-415f-a758-0301d5c8f113.gif)

**개발환경 세팅**

[ROS 환경에서 VLP16 사용하기](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)

```
$ sudo apt-get install ros-melodic-velodyne
$ cd catkin_ws/src
$ git clone https://github.com/ros-drivers/velodyne
$ cd ..
$ catkin_make
$ sudo ifconfig eth0 192.168.3.100
$ cd catkin_ws
$ cd src 
$ roslaunch velodyne_pointcloud VLP16_points.launch
$ rosrun rviz rviz -f velodyne 
```
이때 네트워크 세팅에 주의하세요. 
또한, 네트워크를 세팅할 때 eth0 이 아니라 ifconfig -a 해서 나오는 id 로 설정해야 정상 작동합니다. 

이후에는 rviz 에서 add -> PointCloud2 , topic 탭에서 /velodyne points 탭을 클릭하면 됩니다. 

**data export**

```
$ rosbag record -O velodyne /velodyne_points
$ rosrun pcl_ros bag_to_pcd velodyne.bag /velodyne_points ./pointscloud
```

이렇게 하면 pcl 데이터가 .bag 형태로 저장됩니다. 
