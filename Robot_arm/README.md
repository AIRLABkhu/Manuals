# Robot Arm 

로봇팔을 동작하는 방법에 대한 전반적인 내용을 다룹니다. 

## Table of Contents

1. Trajectory Planning
2. Rviz
3. Gazebo
4. MoveIt
5. Braccio 작동하기 


### Robot Arm Trajectory Planning

로봇팔의 Pick and Place 동작은 움직임 궤적 계획 (Trajectory Planning), 충돌 회피, 역기구학 모델링 (Inverse Kinamatics) 에 대한 연산을 필요로 하며 각각의 연산에 대해 다양한 알고리즘이 개발되어 사용되고 있습니다. 

Path 는 Time scaling 을 통해 Trajectory 로 만들 수 있고, Motion Planning 은 이에 더해 장애물, 환경 등을 더 고려하는 것을 말합니다. 

로봇팔은 회전각 θ 를 제어하는 모터로 구성되고, joint 와 link 의 쌍으로 구성됩니다. 


### MoveIt!

MoveIt 은 운동 계획과 제어를 위한 도구입니다. 

+ ROS MoveIt 패키지 설치
+ URDF (Universal Robotic Description Format) 파일에 로봇팔 기구부의 joint, link, 무게중심, 재질 등을 정의하고 로봇팔 모델링하기 
+ MoveIt Setup Assistant 프로그램을 통해 로봇팔 정보를 설정하기 





