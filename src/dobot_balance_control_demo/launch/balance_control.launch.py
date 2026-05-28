#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("dobot_balance_control_demo")
    default_config = os.path.join(pkg_share, "config", "balance_control_config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="姿态平衡演示 YAML 配置文件路径。",
            ),
            DeclareLaunchArgument(
                "grpc_addr",
                default_value="192.168.5.2:50051",
                description="机器人 gRPC 地址。",
            ),
            DeclareLaunchArgument(
                "record_plot",
                default_value="false",
                description="是否记录姿态数据并在退出时生成 CSV/HTML 曲线结果。",
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_balance_control_demo.imu_reader_node",
                    "--config",
                    LaunchConfiguration("config_file"),
                ],
                name="imu_reader_node",
                output="log",
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_balance_control_demo.balance_controller_node",
                    "--config",
                    LaunchConfiguration("config_file"),
                    "--grpc-addr",
                    LaunchConfiguration("grpc_addr"),
                ],
                name="balance_controller_node",
                output="log",
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_balance_control_demo.demo_monitor_node",
                    "--config",
                    LaunchConfiguration("config_file"),
                ],
                name="demo_monitor_node",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_balance_control_demo.attitude_plot_recorder_node",
                    "--config",
                    LaunchConfiguration("config_file"),
                ],
                name="attitude_plot_recorder_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("record_plot")),
            ),
        ]
    )
