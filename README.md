# (0) 환경 세팅(Jetank 도커 환경 구축)

[jetank1]
도커 설치 및 실행법
-------------------------
```
sudo docker run -it --rm \
    --name=my_bot \
    --net=host \
    --ipc=host \
    --pid=host \
    --privileged \
    -v /dev:/dev \
    -v ~/ros2_ws:/root/ros2_ws \
    ros:humble-ros-base
```
위 명령어 실행 했는데
`Unable to find image 'ros:humble-ros-base' locall` 라는 오류가 뜨면
```
#ros:humble-ros-base 도커를 먼저 다운 받고
sudo docker pull ros:humble-ros-base
```
이거 실행하고 처음거 다시 실행
<img width="188" height="31" alt="image" src="https://github.com/user-attachments/assets/f42ce56a-1196-4f74-8fa5-7e362c1915b3" />
이렇게 바뀌면 성공    

도커 나가려면 
```
exit
```

<br /><br /><br />
도커 쓰기 전에 알아야 할 점
-------------------------
도커는 안에서 내용물 왔다 갔다 수정해도 껐다 키면 까먹음(단 ros2_ws에 따로 파일 만들거나 수정하는건 상관 없음 공유 폴더로 지정한거라)<br/>
그래서 만약에 pip3 install, apt install같은 거 한 후에 저장하고 싶으면    
**도커가 켜져있는 상태로**
다른 터미널에서 ssh접속해서 
```
sudo docker commit my_bot my_jetbot
```
이렇게 하면 기존 ros:humble-ros-base(얘는 인터넷에서 다운받은 도커)도커가  my_jetbot이라는 도커로 이름이 바뀌어 로컬 저장이 됨


<br /><br /><br />
도커 여는 단축키 지정
------------------------------------
도커 열 때 마다
```
sudo docker run -it --rm \
    --name=my_bot \
    --net=host \
    --ipc=host \
    --pid=host \
    --privileged \
    -v /dev:/dev \
    -v ~/ros2_ws:/root/ros2_ws \
    my_jetbot
```
이거 귀찮으니까
```
echo "alias start_jetbot='sudo docker run -it --rm  --name=my_bot --net=host   --ipc=host   --pid=host   --privileged  -v /dev:/dev -v ~/ros2_ws:/root/ros2_ws my_jetbot'" >> ~/.bashrc
source ~/.bashrc
```
이거 한 후 이제 `start_jetbot`칠 때마다 도커 실행됨



도커 만들고 설치해야할 것들
-------------------------------
**-도커안에서-**
```
apt update && apt install python3-pip -y
pip3 install numpy pyserial
apt install python3-opencv -y
apt install i2c-tools -y
pip3 install traitlets
pip3 install Adafruit-MotorHAT
pip3 install ultralytics
```

**-Numpy 버전 맞춰주기-**

```
# 기존 Numpy 삭제하기
python3 -m pip uninstall -y numpy

# Numpy 1.26.4 설치
python3 -m pip install numpy==1.26.4

# 설치 확인(출력이 1.26.4면 정상)
python3 -c "import numpy as np; print(np.__version__); print(np.__file__)"

```

```
cd /root/ros2_ws
mkdir src
cd src
git clone https://github.com/embedded-IEEE/jetank_move_test.git
```

아마 레포가 private이라 로그인 하라 뜰거임 일단 한번 로그인 하고 git clone 하고 
[#1] 
<br/><br/><br/>

```
cd /root/ros2_ws/src/jetank_move_test
. install.sh
```

**위에거 다 설치하고**<br/>
다른 터미널에서 ssh접속해서

```
sudo docker commit my_bot my_jetbot
```


servorinit실행
=============================
**-도커안에서-**
```
cd /root/ros2_ws/src/jetank_move_test/code
python3 servoInt.py
```
위에 파이썬 실행한 후에 바로 일반 컴퓨터 터미널에서
```
ros2 topic echo /robot_state
```
실행하면 토픽 publish/subscribe 확인 가능    

servotest실행
==================================
**도커안에서**
이 코드는 같은 인터넷 망에서 다른 애가 Publish한 걸<br/>
jetank가 subscribe해서 명령 받는 코드<br/><br/><br/>
```
python3 servotest_1.py
```

이후 다른 컴퓨터에서 
```
1.초기 위치
ros2 topic pub --once /jetank_cmd std_msgs/msg/String "data: 'home'"
2.  **팔 이동 (X=150, Y=50 좌표로 이동):**
ros2 topic pub --once /jetank_cmd std_msgs/msg/String "data: 'move 150 50'"
3.  **젯봇 전진:**
ros2 topic pub --once /jetank_cmd std_msgs/msg/String "data: 'forward'"
4.  **젯봇 정지:**
ros2 topic pub --once /jetank_cmd std_msgs/msg/String "data: 'stop'"
```
여기 명령어 보내면 움직임<br/>
<br/>
그리고 움직일 때 얘도 state publish하니까
다른 컴의 다른 터미널에서 
```
ros2 topic echo /robot_state
```

이거 하고 있으면 상태 pub할 수 있음


# (1) 카메라 좌표와 Jetank 월드 좌표 맞춰주기(Homography)
파일 구성 설명
- src: 노트북에서 실행(Orchestration 역할)
- jetank1: Jetank1 내부 도커 환경에서 실행


[노트북]
```
# 연결된 웹캠 번호 확인 -> image_publisher_node_cctv1의 CAM_NUM 설정해주기
v4l2-ctl --list-devices

cd ~/jetank_ws/src/top_cctv_infer/top_cctv_infer

# 연결된 웹캠의 이미지를 ROS 2의 sensor_msgs/Image로 변환해 다른 노드가 사용할 수 있도록 ROS 2 이미지 토픽으로 퍼블리시
python3 image_publisher_node_cctv1.py

# 카메라 화면에서 4점을 클릭하고, 그 점에 대응하는 Jetank의 실제 월드 좌표(x, y)를 사용자가 수동 조작하여 얻은 후 Homography를 계산해 YAML로 저장
python3 latptop_cam_click_calib.py 

```


[jetank1]
=========================

```
# Jetank 접속 후 도커 들어가기
ssh jetbot@IP 주소
start_jetbot

cd /root/ros2_ws/
rm -rf build install log
colcon build 
. install/setup.bash

cd /root/ros2_ws/src/jetank_move_test/code
python3 jetank_calib_move_server.py

```

**(중요!) 여기서 구한 값으로 role_jetank1.py 좌표 넣어주기**
```
cd src/jetank_move_test/code
```
role_jetank1.py의 이 부분에 넣어주면 됨.

```
DEFAULT_PX_POINTS_JETANK1 = "288,430;274,317;155,455;130,329"
DEFAULT_WORLD_POINTS_JETANK1 = "-43.0,160.0;-43.0,100.0;30.0,100.0;30.0,170.0"

```

# (2) Jetank1과 Top_down Camera1 실행
[노트북]

```
cd ~/jetank_ws/src/top_cctv_infer/top_cctv_infer

# 연결된 웹캠의 이미지를 ROS 2의 sensor_msgs/Image로 변환해 다른 노드가 사용할 수 있도록 /jetank/top_cctv1 토픽으로 퍼블리시
python3 image_publisher_node_cctv1.py

# (1) 카메라 이미지(/jetank/top_cctv1)를 받아서 YOLO OBB로 젠가(또는 객체)를 검출
# (2) 가장 가까운 객체를 고르고 annotated 이미지 토픽으로 퍼블리시
# (3) 가장 최근 closest 결과를 서비스(/top_cctv1/get_closest_pose)로 응답
python3 infer_service_server_cctv1.py
```

[jetank1]
```
cd src/jetank_move_test/code

# (1) Top_down Camera1 AI 서비스(/top_cctv1/get_closest_pose)로 “가장 가까운 젠가”의 픽셀 좌표(cx,cy,theta,conf) 를 받아오고
# (2) Homography로 픽셀 좌표를 월드 좌표(x, y, roll)로 바꾼 뒤
# (3) JetankController의 move_to_xyz + 전자석 ON/OFF로 픽/드롭 시퀀스를 수행하는 ROS2 노드(서비스 서버 + 서비스 클라이언트)
python3 role_jetank1.py
```