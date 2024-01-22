"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os


def generate_launch_description():
    # ***** Launch arguments *****

    # 加载一个保存了地图和机器人状态的文件，通常是一个.pbstream格式的文件。这个文件可以用来在ROS中进行纯定位2，
    # 也就是根据已有的地图和机器人的初始位置，进行后续的定位和导航3。
    load_state_filename_arg = DeclareLaunchArgument("load_state_filename")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")  # 是否使用仿真时间

    # ***** File paths ******
    # pkg_share = FindPackageShare("cartographer_ros").find("cartographer_ros")
    pkg_share = get_package_share_directory("fishbot_cartographer")
    urdf_dir = os.path.join(pkg_share, "urdf")
    urdf_file = os.path.join(urdf_dir, "argues_hunter_ackermann.urdf")
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    # ***** Nodes *****
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            # FindPackageShare("cartographer_ros").find("cartographer_ros")
            # + "/configuration_files",
            os.path.join(pkg_share, "config"),
            "-configuration_basename",
            "argus_backpack_2d_localization.lua",
            "-load_state_filename",
            LaunchConfiguration("load_state_filename"),
        ],
        # remappings=[("imu", "argus_ros2/imu_result")],
        # remappings=[("echoes", "horizontal_laser_2d")],
        # remappings=[("echoes", "scan")],
        remappings=[
            ("imu", "/argus_ros2/imu_result"),
            ("scan", "/scan"),
        ],
        output="screen",
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        parameters=[{"use_sim_time": use_sim_time}, {"resolution": 0.05}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        on_exit=Shutdown(),
        arguments=[
            "-d",
            # FindPackageShare("cartographer_ros").find("cartographer_ros")
            # + "/configuration_files/demo_2d.rviz",
            os.path.join(pkg_share, "rviz", "demo_2d.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            # Launch arguments
            declare_use_sim_time_cmd,
            load_state_filename_arg,
            # Nodes
            robot_state_publisher_node,
            cartographer_node,
            cartographer_occupancy_grid_node,
            rviz_node,
        ]
    )
