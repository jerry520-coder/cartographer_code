import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 定位到功能包的地址
    # pkg_share = FindPackageShare(package="fishbot_cartographer").find(
    #     "fishbot_cartographer"
    # )
    cartographer_description_dir = get_package_share_directory("fishbot_cartographer")
    pointcloud_to_laserscan_dir = get_package_share_directory("pointcloud_to_laserscan")
    pointcloud_to_laserscan_launch = os.path.join(pointcloud_to_laserscan_dir, "launch")

    # =====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间，我们用gazebo，这里设置成true
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    # 地图的分辨率
    resolution = LaunchConfiguration("resolution", default="0.05")
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration("publish_period_sec", default="1.0")
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration(
        "configuration_directory",
        default=os.path.join(cartographer_description_dir, "config"),
    )
    # 配置文件
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="fishbot_2d.lua"
    )
    rviz_config_dir = (
        os.path.join(cartographer_description_dir, "rviz") + "/demo_2d.rviz"
    )
    # print(f"rviz config in {rviz_config_dir}")

    ## ***** File paths ******
    # pkg_share = FindPackageShare("cartographer_ros").find("cartographer_ros")
    pkg_share = get_package_share_directory("fishbot_cartographer")
    urdf_dir = os.path.join(pkg_share, "urdf")
    urdf_file = os.path.join(urdf_dir, "argues_hunter_ackermann.urdf")
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    # =====================声明三个节点，cartographer/occupancy_grid_node/rviz_node=================================
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            configuration_directory,
            "-configuration_basename",
            configuration_basename,
        ],
        remappings=[
            ("imu", "/argus_ros2/imu_result"),
            ("scan", "/scan"),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution",
            resolution,
            "-publish_period_sec",
            publish_period_sec,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # 加载通用的sample_pointcloud_to_laserscan_launch文件
    pointcloud_to_laserscan_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pointcloud_to_laserscan_launch,
                "sample_pointcloud_to_laserscan_launch.py",
            )
        )
    )

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    # ===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)
    # ld.add_action(pointcloud_to_laserscan_cmd)
    ld.add_action(robot_state_publisher_node)

    return ld
