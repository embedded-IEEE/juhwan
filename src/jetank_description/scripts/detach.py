#!/usr/bin/env python3

import rclpy
import time
from std_msgs.msg import Empty

def main():
    # 1. ROS 2 초기화
    rclpy.init()
    node = rclpy.create_node('one_shot_detacher')

    # 퍼블리셔들을 담을 리스트
    pubs = []
    
    # ---------------------------------------------------------
    # 설정: (로봇 이름, 젠가 개수)
    # jetank1은 1~10번, jetank2도 1~10번이라고 가정하고 작성했습니다.
    targets = [
        ('jetank1', 10),  # jetank1은 jenga1 ~ jenga10
        ('jetank2', 10)   # jetank2는 jenga1 ~ jenga10 (요청하신 내용 반영)
    ]
    # ---------------------------------------------------------

    # 2. 퍼블리셔 생성 (토픽 연결 준비)
    print("토픽 연결 중...")
    for tank_name, count in targets:
        for i in range(1, count + 1):
            topic_name = f'{tank_name}/jenga{i}/detach'
            # 큐 사이즈 1이면 충분 (한번 쏘고 말 것이므로)
            pub = node.create_publisher(Empty, topic_name, 1)
            pubs.append(pub)

    # [중요] 퍼블리셔와 서브스크라이버(Gazebo)가 서로를 인식할 시간을 줍니다.
    # 이 과정이 없으면 메시지를 보내기도 전에 프로그램이 꺼질 수 있습니다.
    time.sleep(1.0) 

    # 3. 메시지 일괄 전송 (Publish Once)
    msg = Empty()
    print("명령 전송 시작!")
    
    count = 0
    for pub in pubs:
        pub.publish(msg)
        count += 1
        
    print(f"총 {count}개의 젠가에 Detach 명령을 보냈습니다.")
    
    # 4. 종료
    node.destroy_node()
    rclpy.shutdown()
    print("프로그램 종료.")

if __name__ == '__main__':
    main()