from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="scanner",
                default_value="scanner",
                description="Namespace for sample topics",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "-r",
                    "10",  # 设置发布速率为10Hz
                    "--qos-profile",
                    "sensor_data",  # 使用名为"sensor_data"的QoS配置文件
                    [
                        LaunchConfiguration(variable_name="scanner"),
                        "/scan",
                    ],  # 设置主题的命名空间
                    "sensor_msgs/msg/LaserScan",  # 发布的消息类型为LaserScan
                    # 将一个包含激光扫描消息参数的 Python 字典转换为 YAML 格式的字符串
                    yaml.dump(
                        {
                            "header": {"frame_id": "scan"},
                            "angle_min": -1.0,
                            "angle_max": 1.0,
                            "angle_increment": 0.1,
                            "range_max": 10.0,
                            "ranges": [
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                                1,
                            ],  # 发布的激光扫描消息的具体内容
                        }
                    ),
                ],
                name="scan_publisher",  # 进程的名称
            ),
            # 将 "scan" 坐标系相对于 "map" 坐标系进行定义。通过指定平移和四元数来描述两个坐标系之间的相对姿态关系。
            # 这通常用于在 ROS 中建立坐标系之间的静态关系，例如模拟激光雷达相对于地图的位置。
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--qx",
                    "0",
                    "--qy",
                    "0",
                    "--qz",
                    "0",
                    "--qw",
                    "1",  # 没有旋转
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "scan",
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="laserscan_to_pointcloud_node",
                name="laserscan_to_pointcloud",
                remappings=[
                    (
                        "scan_in",
                        [LaunchConfiguration(variable_name="scanner"), "/scan"],
                    ),
                    ("cloud", [LaunchConfiguration(variable_name="scanner"), "/cloud"]),
                ],  # 重新映射主题，将 "scan_in" 映射到用户通过命令行参数设置的命名空间下的 "/scan" 主题，将 "cloud" 映射到 "/cloud" 主题
                parameters=[
                    {"target_frame": "scan", "transform_tolerance": 0.01}
                ],  # 设置节点的参数，目标坐标系为 "scan"，坐标变换容忍度为0.01
            ),
        ]
    )
