#!/usr/bin/env python3
"""Launch Jetank role scripts (except Gazebo)."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _build_role_processes(context, *args, **kwargs):
    role_dir = LaunchConfiguration("role_dir").perform(context)
    device = LaunchConfiguration("device").perform(context)
    roi_xmin = LaunchConfiguration("roi_xmin_ratio").perform(context)
    roi_xmax = LaunchConfiguration("roi_xmax_ratio").perform(context)
    roi_ymin = LaunchConfiguration("roi_ymin_ratio").perform(context)
    roi_ymax = LaunchConfiguration("roi_ymax_ratio").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )

    role_dir = os.path.expanduser(role_dir)

    def _role_path(name: str) -> str:
        return os.path.join(role_dir, name)

    top_cctv1_cmd = [
        "python3",
        _role_path("role_top_cctv1.py"),
        "--ros-args",
        "-p",
        f"device:={device}",
    ]
    top_cctv2_cmd = [
        "python3",
        _role_path("role_top_cctv2.py"),
        "--ros-args",
        "-p",
        f"device:={device}",
        "-p",
        f"roi_xmin_ratio:={roi_xmin}",
        "-p",
        f"roi_xmax_ratio:={roi_xmax}",
        "-p",
        f"roi_ymin_ratio:={roi_ymin}",
        "-p",
        f"roi_ymax_ratio:={roi_ymax}",
    ]
    conveyor_cmd = ["python3", _role_path("role_conveyor.py")]
    jetank1_cmd = ["python3", _role_path("role_jetank1.py")]
    jetank2_cmd = ["python3", _role_path("role_jetank2.py")]
    rc_car_cmd = ["python3", _role_path("role_rc_car.py")]

    if use_sim_time:
        jetank1_cmd.append("--use-sim-time")
        jetank2_cmd.append("--use-sim-time")

    return [
        ExecuteProcess(cmd=top_cctv1_cmd, output="screen"),
        ExecuteProcess(cmd=top_cctv2_cmd, output="screen"),
        ExecuteProcess(cmd=conveyor_cmd, output="screen"),
        ExecuteProcess(cmd=jetank1_cmd, output="screen"),
        ExecuteProcess(cmd=jetank2_cmd, output="screen"),
        ExecuteProcess(cmd=rc_car_cmd, output="screen"),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "role_dir",
                default_value="/home/shin/jetank_ws/src/jetank_move_test/code",
                description="Absolute path to role scripts directory.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="cuda:0",
                description="YOLO device string, e.g. cuda:0 or cpu.",
            ),
            DeclareLaunchArgument(
                "roi_xmin_ratio",
                default_value="0.22",
                description="ROI xmin ratio for top_cctv2 guard.",
            ),
            DeclareLaunchArgument(
                "roi_xmax_ratio",
                default_value="0.40",
                description="ROI xmax ratio for top_cctv2 guard.",
            ),
            DeclareLaunchArgument(
                "roi_ymin_ratio",
                default_value="0.42",
                description="ROI ymin ratio for top_cctv2 guard.",
            ),
            DeclareLaunchArgument(
                "roi_ymax_ratio",
                default_value="0.58",
                description="ROI ymax ratio for top_cctv2 guard.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Pass --use-sim-time to jetank roles.",
            ),
            OpaqueFunction(function=_build_role_processes),
        ]
    )
