# Isaac Gym Installation

Isaac Gym 설치법에 대해서 다루려고 한다. (Ubuntu 20.04 환경에서 설치 되었다.)

## Isaac Gym - Preview Release

https://developer.nvidia.com/isaac-gym 
![스크린샷, 2022-05-03 14-24-11](https://user-images.githubusercontent.com/96813784/166407834-acc2517b-29ed-4842-b19d-43d047ecb59e.png)
이곳에 접속하고 Nvidia Developer에 가입을 해야 한다. 본인이 가지고 있는 구글 계정으로 가입을 진행하자. (Join Now 클릭)

가입을 했다면 member area 가 보이니 그것을 클릭하고 Isaac Gym - Ubuntu Linux 18.04/ 20.04 release를 클릭하여 압축 파일을 다운로드를 받자.
그러면 isaacgym 파일이 보일 것이다. 만약 보인다면 1단계는 끝났다.


##  create_conda_env_rlgpu.sh

![스크린샷, 2022-05-03 14-28-12](https://user-images.githubusercontent.com/96813784/166408133-6058607d-57cd-4543-abc8-09ab6ba06e8a.png)

isaacgym 파일에 들어간 후 terminal를 열고 $./create_conda_env_rlgpu.sh 를 입력하고 isaacgym conda env가 구축되기를 기다리자.

![스크린샷, 2022-05-03 14-31-23](https://user-images.githubusercontent.com/96813784/166408298-20c6d7fb-8505-4408-bf71-0055902a3be3.png)

## Testing the installation


![스크린샷, 2022-05-03 14-35-01](https://user-images.githubusercontent.com/96813784/166408601-5f65a419-dfb6-41ef-bfbf-8110e7aa246f.png)
다시 isaacgym 파일에 들어가자. 그리고 python sub 파일에 들어간 후 거기서 terminal을 열어 아래 명령어를 입력하자
```
export LD_LIBRARY_PATH=/home/user_name/anaconda3/envs/rlgpu/lib
conda activate rlgpu
cd examples
python joint_monkey.py
```
Anaconda로 Isaac Gym을 Download을 하였을 때 라이브러리가 파이썬 경로를 제대로 찾지 못하는 문제가 발생하는데 NVIDIA 에서는 export로 경로를 설정하면 문제가 해결된다고 한다. $export LD_LIBRARY_PATH=/home/user_name/anaconda3/envs/rlgpu/lib  **그대로 복사하는 게 아니라 user_name 부분은 자신의 linux computer의 user_name**

![스크린샷, 2022-05-03 14-35-38](https://user-images.githubusercontent.com/96813784/166408604-f5f489a9-f789-45c5-b8d4-97421519bf7f.png)
![스크린샷, 2022-05-03 14-35-49](https://user-images.githubusercontent.com/96813784/166408607-9df9b47a-a40b-4719-b264-edb7f482abb0.png)
