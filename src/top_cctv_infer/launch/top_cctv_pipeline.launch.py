#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # 1) image_publisher_node
    image_pub = Node(
        package="top_cctv_infer",
        executable="image_publisher_node",
        name="image_publisher_node",
        output="screen",
        # parameters=[{"cam_num": 0, "pub_topic": "/jetank/top_cctv1"}],  # 필요하면 주석 해제
    )

    # 2) obb_inference_node
    obb_infer = Node(
        package="top_cctv_infer",
        executable="obb_inference_node",
        name="obb_inference_node",
        output="screen",
        # parameters=[{"image_topic": "/jetank/top_cctv1"}],
    )

    # 3) infer_node
    infer_node = Node(
        package="top_cctv_infer",
        executable="infer_node",
        name="infer_node",
        output="screen",
    )

    # 4) rqt_image_view
    rqt_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
        # arguments=["--ros-args", "-r", "image:=/top_cctv/annotated"],  # 보고 싶은 토픽으로 바꿔서 사용
    )

    # “이 순서대로” 실행되도록 지연 실행(초)
    return LaunchDescription(
        [
            image_pub,
            TimerAction(period=1.0, actions=[obb_infer]),
            TimerAction(period=4.0, actions=[infer_node]),
            TimerAction(period=10.0, actions=[rqt_view]),
        ]
    )
