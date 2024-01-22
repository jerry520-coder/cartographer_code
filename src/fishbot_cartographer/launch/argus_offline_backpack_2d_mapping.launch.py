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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("fishbot_cartographer")
    pointcloud_to_laserscan_dir = get_package_share_directory("pointcloud_to_laserscan")
    pointcloud_to_laserscan_launch = os.path.join(pointcloud_to_laserscan_dir, "launch")

    ## ***** Launch arguments *****
    bag_filenames_arg = DeclareLaunchArgument("bag_filenames")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")  # 是否使用仿真时间

    no_rviz_arg = DeclareLaunchArgument("no_rviz", default_value="false")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        # default_value=FindPackageShare("cartographer_ros").find("cartographer_ros")
        # + "/configuration_files/demo_2d.rviz",
        default_value=os.path.join(pkg_share, "rviz", "demo_2d.rviz"),
    )

    configuration_directory_arg = DeclareLaunchArgument(
        "configuration_directory",
        # default_value=FindPackageShare("cartographer_ros").find("cartographer_ros")
        # + "/configuration_files",
        default_value=os.path.join(pkg_share, "config"),
    )

    configuration_basenames_arg = DeclareLaunchArgument(
        "configuration_basenames", default_value="argus_backpack_2d_mapping.lua"
    )

    urdf_filenames_arg = DeclareLaunchArgument(
        "urdf_filenames",
        # default_value=FindPackageShare("cartographer_ros").find("cartographer_ros")
        # + "/urdf/backpack_2d.urdf",
        default_value=os.path.join(pkg_share, "urdf", "argues_hunter_ackermann.urdf"),
    )

    ######################################################################################################################

    ## ***** Nodes *****
    offline_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # FindPackageShare("cartographer_ros").find("cartographer_ros")
            # + "/launch/offline_node.launch.py"
            os.path.join(pkg_share, "launch", "argus_offline_node.launch.py")
        ),
        launch_arguments={
            "bag_filenames": LaunchConfiguration("bag_filenames"),
            "no_rviz": LaunchConfiguration("no_rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "configuration_directory": LaunchConfiguration("configuration_directory"),
            "configuration_basenames": LaunchConfiguration("configuration_basenames"),
            "urdf_filenames": LaunchConfiguration("urdf_filenames"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # scan_remap = SetRemap("scan", "echoes")
    # imu_remap = SetRemap("imu", "argus_ros2/imu_result")
    imu_remap = SetRemap("argus_ros2/imu_result", "imu")

    # 加载通用的sample_pointcloud_to_laserscan_launch文件
    pointcloud_to_laserscan_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pointcloud_to_laserscan_launch,
                "sample_pointcloud_to_laserscan_launch.py",
            )
        )
    )
    return LaunchDescription(
        [
            # Launch arguments
            bag_filenames_arg,
            declare_use_sim_time_cmd,
            no_rviz_arg,
            rviz_config_arg,
            configuration_directory_arg,
            configuration_basenames_arg,
            urdf_filenames_arg,
            # Nodes
            # scan_remap,
            imu_remap,
            offline_node_launch,
            # pointcloud_to_laserscan_cmd,
        ]
    )
