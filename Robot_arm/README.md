# Robot Arm

## Intro

로봇팔에 대한 전반적인 내용과, 
Braccio 로봇팔로 수행하는 pick and place 를 다룹니다. 

## 결과 동영상

1. 박스를 쌓는 로봇팔

[![braccio stack Youtube](https://img.youtube.com/vi/yRtQsGaJM6Y/maxresdefault.jpg)](https://youtu.be/yRtQsGaJM6Y)


### Week1 

#### 1,2,3 Section 별로 keyboard 입력 받아서 움직이는 Gazebo Simulator 

아래쪽 판자를 왼쪽에서부터 1,2,3 이라고 하여 <br>
키보드 입력으로 (1 2) , (1 3) 등을 입력하면 1번 판자로부터 2번 판자로, 1번 판자로부터 3번 판자까지 pick and place 를 수행하는 simulator 입니다. 

![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/52185595/155872869-de41b96b-9cb2-417c-86d2-3e8801ac0167.gif)

#### 1,2,3 Section 별로 keyboard 입력 받아서 움직이는 Braccio Arm 

![ezgif com-gif-maker (2)](https://user-images.githubusercontent.com/52185595/155873061-0f489262-2420-462e-86d9-2bef8fe288d3.gif)

### Week2

#### xy 평면 상에서 시작점, 끝점 좌표 입력 받아서 움직이는 Gazebo Simulator

키보드 입력으로 start x, start y, end x, end y 값을 받아 움직이는 시뮬레이터입니다. 

![ezgif com-gif-maker (6)](https://user-images.githubusercontent.com/52185595/155873190-488e1548-3512-42d0-97f2-1168158cb01c.gif)

#### Simulator 와 동일하게 움직이는 Braccio Arm 

![ezgif com-gif-maker (9)](https://user-images.githubusercontent.com/52185595/155912569-702d7a64-c3b0-4719-9fe5-b43d1f7d7cc0.gif)

##### Joint State Publisher

ROS 의 Joint State Publisher 를 이용해서 Rviz 화면과 동일하게 움직이는 braccio 입니다.

![ezgif com-gif-maker (8)](https://user-images.githubusercontent.com/52185595/155873317-11bca673-788a-4112-aba9-2e3099d3f077.gif)

### Week3

#### 물체를 들어올려 쌓을 수 있는 Gazebo Simulator



## 목차

1. 로봇팔 개요<br>
	1-1. 자주 나오는 용어 설명<br>
    1-2. Inverse Kinematics<br>
    1-3. MoveIt!<br>
    1-4. Braccio Robot Arm <br>
2. Braccio 로 pick and place 수행하기<br> 
	2-1. 브라키오에서 ROS 를 사용할 수 있게 세팅하기<br>
	2-2. 상자를 keyboard input 에 따라 옮기기<br>
    2-3. 상자를 쌓기 <br>

### 1. 로봇팔 개요

#### 1-1. 자주 나오는 용어 설명

- **URDF**
Unified Robot Description Format
로봇 모델에 대한 정보들을 기술해 놓은 규격서. 

- **MoveIt**
매니퓰레이터를 위한 통합 라이브러리. 
모션 플래닝을 위한 빠른 역기구학 해석, 매니퓰레이션을 위한 고급 알고리즘, 로봇 핸드 제어, 동역학, 제어기 등 다양한 기능을 제공하고 있다. 

- **Forward Kinematics**
로봇의 각 관절 각도가 주어졌을 때 말단 장치의 위치 및 자세를 구하는 것

- **Inverse Kinematics**
말단 장치의 위치와 자세가 주어졌을 때 각 관절 각도를 구하는 것. 

#### 1-4. Braccio Robot Arm

<img src="https://images.velog.io/images/zzziito/post/a9c5d984-5389-4d96-a58a-a5b74be6f33b/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202022-02-28%20%EC%98%A4%ED%9B%84%202.58.08.png" width="300" height="400">

브라키오는 6축 로봇팔입니다. 

### 2. Braccio 로 pick and place 수행하기 


#### 2-1. 브라키오에서 ROS 를 사용할 수 있게 세팅하기

<img src="https://images.velog.io/images/zzziito/post/2701d48d-7aec-4247-8c0a-bd0f8e8b2df3/IMG_9094.jpg" width="300" height="300">

브라키오는 다음과 같이 아두이노 우노와 부착된 Shield 로 구성되어 있습니다. 

![](https://images.velog.io/images/zzziito/post/2f776030-860c-4e47-895e-2855d9cfc9ae/HardwareArchitecture.jpeg)

브라키오는 다음과 같이 아두이노와 쉴드에 부착되어 있습니다. 이 아두이노가 ROS 를 사용할 수 있게 하려면, 따로 설정을 해 주어야 합니다. 
Setup 에 관해서는 [이곳](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) 을 참고해 주세요. 
이후 braccio/braccio_ros.ino 파일을 아두이노에 업로드해주세요. 

```
Error opening serial : could not open port /dev/ttyACM0 Permission denied 
```
라는 에러가 떴다면 

```
$ sudo chmod 666 /dev/ttyACM0
```

를 입력하여 포트를 모든 사용자가 사용할 수 있도록 권한을 부여해 주세요. 


#### Code

이 부분은 코드 설명을 다룹니다. 

```python 
import sys
import rospy
import moveit_commander
import time
from gazebo_msgs.msg import LinkStates, ModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
```

```python

```


#### Inverse Kinematics

브라키오는 6축 로봇팔이지만, 이 프로젝트에서는 하단부 회전축 (j0), 중간부 (j1,j2,j3), 고정된 축 하나(j4), 그리퍼로 나누어서 다룹니다. 따라서 inverse kinematics 는 3축을 다룬다고 할 수 있겠습니다. 

<img width="535" alt="스크린샷 2022-02-28 오후 2 58 08" src="https://user-images.githubusercontent.com/52185595/155931884-564b176e-c7cd-4980-9a71-fa398cb73316.png">

하단부 회전축은 j0 변수로 따로 다루고, 그리퍼는 일단은 로봇과 수직하게 놓여있는 물체만 잡는다는 가정 하에 시뮬레이션 했기 때문에 j4 축은 고정하였습니다. 또한 그리퍼는 open_gripper, close_gripper 라는 함수를 만들어서 따로 다루었습니다.

```python
def go_to_j(self, j0=None, j1=None, j2=None, j3=None):
  joint_goal = self.move_group.get_current_joint_value()
  if j0 is not None:
    joint_goal[0] = j0
  if j1 is not None:
    joint_goal[1] = j1
  if j2 is not None:
    joint_goal[2] = j2
  if j3 is not None:
    joint_goal[3] = j3
  self.go_to_joint(joint_goal)
  
def go_to_joint(self, joint_targets):
  joint_goal = self.move_group.get_current_joint_values()
  joint_goal[0] = joint_targets[0]
  joint_goal[1] = joint_targets[1]
  joint_goal[2] = joint_targets[2]
  joint_goal[3] = joint_targets[3]
  joint_goal[4] = 1.5708
  ret = self.move_group.go(joint_goal, wait=True)
  self.move_group.stop()
```


```python
def inv_kin(self, x, min_y, max_y, end_angle):
  def distance_to_default(q,x):
    x = (self.L[0]*np(cos(q[0]) + self.L[1]*np.cos(q[0]+q[1])+self.L[2]*np.sin(np.sum(q)))
    return self.max_y - y
  
  def y_upper_constraint(q, *args):
    y = (self.L[0]*np.sin(q[0] + self.L[1]*np.sin(q[0]+q[1]) + self.L[2]*np.sin(np.sum(q)))
    return y - self.min_y
  
  def y_lower_constraint(q, *args):
    y = (self.L[0]*np.sin(q[0] + self.L[1]*np.sin(q[0]+q[1]) + self.L[2]*np.sin(np.sum(q)))
      return y - self.min_y
   
  def joint_limits_upper_constraint(q, *args):
    return self.max_angles - q
  
  def joint_limits_lower_constraint(q, *args):
    return q - self.min_angles
  
  def joint_limits_last_orientation(q, *args):
    return self.end_angle_tol - np.abs(np.sum(q)-self.end_angle)
    
  self.min_y = min_y
  self.max_y = max_y
  if end_angle is not None:
    self.end_angle = end_angle
  q = scipy.optimize.fmin_slsqp(func = distance_to_default, x0 = self.q, args(x,), iprint = 0, ieqcons = [joint_limits_last_orientation, joint_limits_upper_constraint, joint_limits_lower_constraint, y_upper_constraint, y_lower_constraint])
  self.q = q
  return self.q
   
  
```

#### ROS Message

```python
rospy.Subscriber("/gazebo/link_states", LinkStates, self.linkstate_callback)
```

### Functions

**reset_target_position**

이 함수는 임의로 빨간 상자의 위치를 변경시키는 함수입니다. 

```python
def reset_target_position(self):
  x = raw_input()
  y = raw_input()
  z = raw_input()
  self.reset_link('unit_box_0', x,y,z)
def reset_link(seflf, name, x, y, z):
  state_msg = ModelState()
  state_msg.model_name = name
  state_msg.pose.position.x = float(x)
  state_msg.pose.position.y = float(y)
  state_msg.pose.position.z = float(z)
  state_msg.pose.orientation.x = 0
  state_msg.pose.orientation.y = 0
  state_msg.pose.orientation.z = 0
  state_msg.pose.orientation.w = 0
  rospy.wait_for_service('/gazebo/set_model_state')
  try:
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(state_msg)
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e
```

**get_box_position**

이 함수는 빨간 상자의 위치를 결과값으로 return 하는 함수입니다. 

```python
def get_box_position(self):
  x, y, r = self.get_link_position(['unit_box_0::link'])
  return self.transform(x,y,r)

def get_link_position(self, link_names):
  x = 0
  y = 0
  n = 0
  for l in link_names:
    ind = self.linkstate_data.name.index(l)
    res = self.linkstate_data.pose[ind].position
    x += res.x
    y += res.y
    n += 1
  return x/n, y/n, DEFAULT_ROT
```

**go_to_targets**

```python
def go_to_target(self, how):
  x,y,r = self.get_box_position()
  returun self.go_to_xy(x,y,r,how)
  

  
```
