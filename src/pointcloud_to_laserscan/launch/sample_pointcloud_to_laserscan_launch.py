from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = DeclareLaunchArgument(
        name="scanner",
        default_value="",
        description="Namespace for sample topics",
    )

    pointcloud_publisher_node = Node(
        package="pointcloud_to_laserscan",
        executable="dummy_pointcloud_publisher",
        remappings=[
            ("cloud", [LaunchConfiguration(variable_name="scanner"), "/cloud"])
        ],  # 重新映射主题，将 "cloud" 映射到用户通过命令行参数设置的命名空间下的 "/cloud" 主题
        parameters=[
            {"cloud_frame_id": "cloud", "cloud_extent": 2.0, "cloud_size": 500}
        ],  # 设置节点的参数，包括点云的坐标系、范围和大小
        name="cloud_publisher",
    )

    tf2_publisher_node = Node(
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
            "1",
            "--frame-id",
            "map",  # map
            "--child-frame-id",
            "hesai_lidar",  # cloud
        ],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[
            (
                "cloud_in",
                [LaunchConfiguration(variable_name="scanner"), "/lidar_points"],
            ),
            ("scan", [LaunchConfiguration(variable_name="scanner"), "/scan"]),
        ],
        parameters=[
            {
                "target_frame": "hesai_lidar",  # 目标坐标系 cloud
                "transform_tolerance": 0.01,  # 0.01
                "min_height": -0.2,  # 点云的最小高度 0.0
                "max_height": 0.2,  # 点云的最大高度 1.0
                "angle_min": -3.14,  # -M_PI/2 扫描角度的最小值 -1.5708
                "angle_max": 0.0,  # M_PI/2 扫描角度的最大值 1.5708
                "angle_increment": 0.01396,  # M_PI/360.0 扫描角度的增量 0.0087  #0.8du
                "scan_time": 0.1,  # 扫描的时间间隔 0.3333  #10hz
                "range_min": 0.5,  # 扫描范围的最小值 0.45
                "range_max": 200.0,  # 扫描范围的最大值 4.0
                "use_inf": False,  # 是否使用无穷大值 True
                "inf_epsilon": 1.0,  # 1.0 无穷大值的替代值 scan_msg->range_max + inf_epsilon_
            }
        ],
        name="pointcloud_to_laserscan",
    )

    ld = LaunchDescription()

    ld.add_action(namespace)
    # ld.add_action(pointcloud_publisher_node)
    # ld.add_action(tf2_publisher_node)
    ld.add_action(pointcloud_to_laserscan_node)

    return ld
