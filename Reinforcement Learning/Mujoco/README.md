# Mujoco Installation 

강화학습 연구 실험에 많이 사용되는 시뮬레이션 중에 하나인 Mujoco-py를 설치하는 방법에 대해 다룰 계획이다.
설치 환경은 Ubuntu 20.04이며 16.04 시리즈 이상이면 모두 설치가 가능하다.

# Install Anaconda

Mujoco-py를 Anaconda virtual-env에 라이브러리로 넣을 것이기 때문에 Linux로 Anaconda를 설치한다. 


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


**#conda initialize**위와 **#enable programmable completion** 아래 사이 빈공간에

![스크린샷, 2022-05-03 12-06-31](https://user-images.githubusercontent.com/96813784/166400269-d64a739d-4bb2-4f4f-9f4d-03c42281e4cb.png)

<pre><code>export LD_LIBRARY_PATH=/home/user_name/.mujoco/mujoco210/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export PATH="$LD_LIBRARY_PATH:$PATH"
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
</code></pre>

이걸 전체 복사해서 붙여넣기 하자. 여기서 use_name은 본인의 우분투 username을 입력해야 한다.
![image](https://user-images.githubusercontent.com/96813784/166399126-0f37ad9d-d59c-4e08-bfc1-dceae152e402.png)

자 보통은 이러면 무조건 에러가 난다. 왜?? 대부분의 컴퓨터에 libGLEW가 없을 것이기 때문이다.

이유는 두 가지다.

첫 번째 이유 : Nvidia Driver와 Cuda가 없다.
이런 경우는 검색을 해서 자신의 그래픽 카드에 맞는 Nvidia Driver와 Cuda를 설치하고 오자.

두 번째 이유: Nividia Driver와 Cuda는 설치가 되었지만 libGLEW가 설치되지 않아 preload가 잡히지 않는 경우다.

대부분 두 번째 이유로 MuJoCo-py가 실행되지 않을 것이다. 이때는 LibGLEW.so를 설치해주고 LD_PRELOAD에 해당 경로를 지정해주면 된다.

$ sudo apt install libglew-dev libgl-dev

terminal에서 위 명령어를 입력하고 LibGLEW.so를 다운 받자. 그리고

$ echo LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGL.so:/usr/lib/x86_64-linux-gnu/libGLEW.so를 입력해 해당 경로를 지정해 주자.

## source .bashrc

![image](https://user-images.githubusercontent.com/96813784/166399720-ee7c95c4-50ae-49eb-8c28-ae297ae1ab8b.png)

이제 Terminal를 켜고 source .bashrc를 입력해서 저렇게 문제없으면 제대로 설치하였다. 만약 제대로 되지 않았다면 4번 과정에서 문제가 발생했을 가능성이 높으니 다시 차근차근 설치해 보길 바란다.

# Install mujoco-py

이제 Mujoco-py를 담으려고 만들었던 가상 환경에 들어가서 명령어를 하나하나 입력하자
(단, 아래 코드에서 conda install -c pytorch pytorch=1.5 torchvision=0.6.1 cudatoolkit=10.2 부분은 Pytorch 홈페이지를 통해 자신의 GPU CUDA version에 맞게 설치해야 한다.)

<pre><code>conda create --name mujoco_py python=3.8
conda activate mujoco_py
conda install -c pytorch pytorch=1.5 torchvision=0.6.1 cudatoolkit=10.2
sudo apt update
sudo apt-get install patchelf
sudo apt-get install python3-dev build-essential libssl-dev libffi-dev libxml2-dev
sudo apt-get install libxslt1-dev zlib1g-dev libglew1.5 libglew-dev python3-pip

git clone https://github.com/openai/mujoco-py
cd mujoco-py
pip install -r requirements.txt
pip install -r requirements.dev.txt

pip3 install -e . --no-cache
</code></pre>


# Reboot your machine

# Run these commands

<pre><code>conda activate mujoco_py
cd mujoco-py
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3
sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so
pip3 install -U 'mujoco-py<2.2,>=2.1'
cd examples
python3 setting_state.py
</code></pre>

![image](https://user-images.githubusercontent.com/96813784/166399807-ad7f5584-b91b-4e5b-8196-77f50faa62a9.png)

최종적으로 이게 나온다면 Mujoco-py를 제대로 설치한 것이다
설치법을 정리한 PDF 파일
[mujoco_installation.pdf](https://github.com/AIRLABkhu/Manuals/files/8608089/mujoco_installation.pdf)
