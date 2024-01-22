#!/bin/bash

function fishbotCartographer() {
    ros2 launch fishbot_cartographer cartographer.launch.py \
        use_sim_time:=True \
        configuration_basename:=fishbot_2d.lua
}

function pointcloud_to_laserscan() {
    ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py
}

function argus_fm_mapping() {
    ros2 launch fishbot_cartographer argus_offline_backpack_2d_mapping.launch.py \
        bag_filenames:=/home/binghe/file_mount_point/0118/0118/0118_0.db3 \
        use_sim_time:=True

    # bag_filenames:=/home/binghe/file_mount_point/0115/0115_0.db3 \
}

function argus_fl_loc() {
    ros2 launch fishbot_cartographer argus_offline_backpack_2d_localization.launch.py \
        bag_filenames:=/home/binghe/file_mount_point/0118/0118/0118_0.db3 \
        load_state_filename:=/home/binghe/orbbec_code/cartographer_ros2/src/fishbot_cartographer/map/0118_0.db3.pbstream \
        use_sim_time:=True
}

function argus_nm_mapping() {
    ros2 launch fishbot_cartographer argus_online_backpack_2d_mapping.launch.py \
        use_sim_time:=True
}

function argus_nl_loc() {
    ros2 launch fishbot_cartographer argus_online_backpack_2d_localization.launch.py \
        use_sim_time:=True \
        load_state_filename:=/home/binghe/orbbec_code/cartographer_ros2/src/fishbot_cartographer/map/0118_0.db3.pbstream
}

function usage() {
    echo -e "\033[33m Usage:
        f           -> run fishbotCartographer node.
        p           -> run pointcloud_to_laserscan node.
        fm           -> run argus_fm_mapping.
        fl           -> run argus_fl_loc.
        nm           -> run argus_nm_mapping.
        nl           -> run argus_nl_loc.
        \033[0m"
}

if [ -z $1 ]; then
    usage
else
    case $1 in
    f) fishbotCartographer ;;
    p) pointcloud_to_laserscan ;;
    fm) argus_fm_mapping ;;
    fl) argus_fl_loc ;;
    nm) argus_nm_mapping ;;
    nl) argus_nl_loc ;;
    # cap) capture ;;
    # rmd) rmdriver ;;
    # all) runAllSensor ;;
    # launch) launchMain ;;
    *) usage ;;
    esac
fi
