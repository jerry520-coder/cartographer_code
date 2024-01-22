
```bash
ros2 run nav2_map_server map_saver_cli -t map -f fishbot_map
#   -t <map_topic>
#   -f <mapname>

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/binghe/orbbec_code/cartographer_code/src/fishbot_cartographer/map0115.pbstream'}"
```

