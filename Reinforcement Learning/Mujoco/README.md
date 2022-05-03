# Mujoco Installation 

강화학습 연구 실험에 많이 사용되는 시뮬레이션 중에 하나인 Mujoco-py를 설치하는 방법에 대해 다룰 계획이다.
설치 환경은 Ubuntu 20.04이며 16.04 시리즈 이상이면 모두 설치가 가능하다.

# Install Anaconda

Mujoco-py를 Anaconda virtual-env에 라이브러리로 넣을 것이기 때문에 Linux로 Anaconda를 설치한다. Anaconda 설치법은 이 글에서는 다루지 않을 것이다. (인터넷에 많이 설치법이 많이 소개되어 있고 설치법이 쉽다.) 

+ Create and activate a conda environment
<pre><code>conda create -n transt python=3.7
conda activate mujoco</code></pre>

+ Install PyTorch
 <pre><code>conda install -c pytorch pytorch=1.5 torchvision=0.6.1 cudatoolkit=10.2
</code></pre>


# Install the Mujoco Library

## Mujoco Library Download

 <pre><code>https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz
</code></pre>
이 주소에 들어가 Mujoco 압축파일을 다운로드 받자.

## Create a Hidden Folder

terminal를 켜고 mkdir/home/username/.mujoco 를 입력해 mujoco 파일을 담을 Hidden Folder를 하나 만들자. Hidden folder가 보이지 않는 사람은 파일 홈에 들어간 후 숨긴 파일 표시를 누르면 숨겨진 .mujoco 파일을 발견할 수 있다.

![image](https://user-images.githubusercontent.com/96813784/166398681-1bd5dc9b-1c5f-4450-af1c-182354dae1bf.png)

## Extract the library to the .mujoco folder

아까 다운 받았던 Mujoco 압축 파일을 .mujoco에 압축 해제하자.
![image](https://user-images.githubusercontent.com/96813784/166398863-734143fb-8adf-42e3-8ba6-9f6cbb7d0de8.png)
![image](https://user-images.githubusercontent.com/96813784/166398875-393418b5-d0cf-4bb6-a1a1-4b41eaa75c16.png)

이렇게 압축 해제를 완료했다면 우선 첫 단계는 성공적으로 완료하였다.

## Include these Lines in .bashrc file

**이 부분이 MuJoCo 설치에서 제일 많이 에러가 나는 부분이고 여기에서 문제가 없으면 거의 다 설치했다고 보면 된다.**

우선 파일/홈으로 들어가서
![image](https://user-images.githubusercontent.com/96813784/166399076-3cb9c716-0ea3-4acc-9615-789ee52e2a6f.png)
.bashrc 파일을 클릭해 들어가자 그리고 쭉 내려서
![image](https://user-images.githubusercontent.com/96813784/166399126-0f37ad9d-d59c-4e08-bfc1-dceae152e402.png)

**conda initialize**위와 **enable programmable completion** 사이에

<pre><code>export LD_LIBRARY_PATH=/home/user_name/.mujoco/mujoco210/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export PATH="$LD_LIBRARY_PATH:$PATH"
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
</code></pre>
