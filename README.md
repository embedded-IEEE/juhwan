# Jetank1 환경 세팅 & Top-CCTV1 연동 가이드

<br/>

Jetank 로봇(도커 환경)과 Top-Down Camera 기반 AI 인식을 연동하여  
젠가 픽/드롭 자동화를 수행하기 위한 **전체 실행 가이드**이다.

---

<br/>

## 전체 실행 순서

1. Jetank1에서 Docker 환경 구축  
2. 필수 패키지 설치 후 Docker 이미지 저장(commit)  
3. 카메라 ↔ Jetank 월드 좌표 Homography 캘리브레이션  
4. Top-CCTV1 AI 서비스 실행  
5. Jetank1 실행  

---

<br/>

## 프로젝트 디렉터리 & 파일 구조 설명

### [노트북]

#### src (노트북)
- 카메라 입력 퍼블리시
- YOLO 기반 객체 인식
- AI 서비스 제공
- Homography 캘리브레이션
```
ros2_ws/
└── src/
    └── top_cctv_infer/
        └── top_cctv_infer/
            ├── image_publisher_node_cctv1.py
            ├── infer_service_server_cctv1.py
            ├── latptop_cam_click_calib.py
            └── ...

```

<br/>


### [Jetank1]

#### jetank1 (Jetank 도커)
- 로봇 팔/전자석 제어
- 좌표 변환
- 실제 픽 & 드롭 수행

```
jetank_ws/
└── src/
    └── jetank_move_test/
        └── code/
            ├── role_jetank1.py
            ├── role_jetank_controller.py
            ├── role_ai_utils.py
            ├── role_dds_config.py
            ├── jetank_calib_move_server.py
            ├── servoInt.py
            ├── servotest_1.py
            ├── magnet_driver.py
            ├── motor.py
            ├── motorfront.py
            └── ...
```
---

<br/>

## 요구사항

- ROS 2 Humble
- Docker
- Jetank 로봇
- 노트북 + USB 웹캠
- 노트북 ↔ Jetank **같은 네트워크**

---

<br/><br/>


# (0) Docker 환경 세팅 (Jetank1)

## 0-1. Docker 컨테이너 실행 (최초)

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
### 위 명령어 실행 했는데 `Unable to find image 'ros:humble-ros-base' locall` 라는 오류가 뜨면
```
#ros:humble-ros-base 도커를 먼저 다운 받고
sudo docker pull ros:humble-ros-base
```
이거 실행하고 처음거 다시 실행
<img width="188" height="31" alt="image" src="https://github.com/user-attachments/assets/f42ce56a-1196-4f74-8fa5-7e362c1915b3" />
이렇게 바뀌면 성공    

<br/>

## 0-2. Docker 컨테이너 종료

```
exit
```
---
<br/>

## 0-3. Docker 이미지 저장 (중요)
#### 도커 쓰기 전에 알아야 할 점
-------------------------
도커는 안에서 내용물 왔다 갔다 수정해도 껐다 키면 까먹음(단 ros2_ws에 따로 파일 만들거나 수정하는건 상관 없음 공유 폴더로 지정한거라)<br/>
그래서 만약에 pip3 install, apt install같은 거 한 후에 저장하고 싶으면    
**도커가 켜져있는 상태로**
다른 터미널에서 ssh접속해서 
```
sudo docker commit my_bot my_jetbot
```
이렇게 하면 기존 ros:humble-ros-base(얘는 인터넷에서 다운받은 도커)도커가  my_jetbot이라는 도커로 이름이 바뀌어 로컬 저장이 됨

---

<br/>

## 0-4. Docker 실행 단축키(alias)
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

---

<br/>

## 0-5. Docker 내부 필수 패키지 설치
```
start_jetbot
```
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

### Numpy 버전 고정

```
# 기존 Numpy 삭제하기
python3 -m pip uninstall -y numpy

# Numpy 1.26.4 설치
python3 -m pip install numpy==1.26.4

# 설치 확인(출력이 1.26.4면 정상)
python3 -c "import numpy as np; print(np.__version__); print(np.__file__)"

```

<br/>

## 0-6. Jetank 코드 클론

```
cd /root/ros2_ws
mkdir src
cd src
git clone https://github.com/embedded-IEEE/jetank_move_test.git
```

아마 레포가 private이라 로그인 하라 뜰거임 일단 한번 로그인 하고 git clone 하고 
```
cd /root/ros2_ws/src/jetank_move_test
. install.sh
```

**위에거 다 설치하고**<br/>
다른 터미널에서 ssh접속해서

```
sudo docker commit my_bot my_jetbot
```

---
<br/><br/><br/>

# (1) 카메라 좌표 ↔ Jetank 월드 좌표 (Homography)
파일 구성 설명
- src: 노트북에서 실행(Orchestration 역할)
- jetank1: Jetank1 내부 도커 환경에서 실행


## [노트북]
```
# 연결된 웹캠 번호 확인 -> image_publisher_node_cctv1의 CAM_NUM 설정해주기
v4l2-ctl --list-devices

cd ~/jetank_ws/src/top_cctv_infer/top_cctv_infer

# 연결된 웹캠의 이미지를 ROS 2의 sensor_msgs/Image로 변환해 다른 노드가 사용할 수 있도록 ROS 2 이미지 토픽으로 퍼블리시
python3 image_publisher_node_cctv1.py

# 카메라 화면에서 4점을 클릭하고, 그 점에 대응하는 Jetank의 실제 월드 좌표(x, y)를 사용자가 수동 조작하여 얻은 후 Homography를 계산해 YAML로 저장
python3 latptop_cam_click_calib.py 

```

## [jetank1]

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

<br/><br/><br/>

# (2) Jetank1과 Top_down Camera1 실행
## [노트북]

```
cd ~/jetank_ws/src/top_cctv_infer/top_cctv_infer

# 연결된 웹캠의 이미지를 ROS 2의 sensor_msgs/Image로 변환해 다른 노드가 사용할 수 있도록 /jetank/top_cctv1 토픽으로 퍼블리시
python3 image_publisher_node_cctv1.py

# (1) 카메라 이미지(/jetank/top_cctv1)를 받아서 YOLO OBB로 젠가(또는 객체)를 검출
# (2) 가장 가까운 객체를 고르고 annotated 이미지 토픽으로 퍼블리시
# (3) 가장 최근 closest 결과를 서비스(/top_cctv1/get_closest_pose)로 응답
python3 infer_service_server_cctv1.py
```

## [jetank1]
```
cd src/jetank_move_test/code

# (1) Top_down Camera1 AI 서비스(/top_cctv1/get_closest_pose)로 “가장 가까운 젠가”의 픽셀 좌표(cx,cy,theta,conf) 를 받아오고
# (2) Homography로 픽셀 좌표를 월드 좌표(x, y, roll)로 바꾼 뒤
# (3) JetankController의 move_to_xyz + 전자석 ON/OFF로 픽/드롭 시퀀스를 수행하는 ROS2 노드(서비스 서버 + 서비스 클라이언트)
python3 role_jetank1.py
```