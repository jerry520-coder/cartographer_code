#!/usr/bin/env bash

set -e # 设置脚本在任何命令返回非零退出状态时立即退出

print_usage() {
  echo "  Usage: $0 [-d] [-r] [-dp] [-rp] [-h]"
}

print_help() {
  echo
  print_usage
  # echo
  # echo "  Options: "
  # echo
  # echo "    -d, Debug"
  # echo "    -r, Release"
  # echo "    -dp, Debug parallel"
  # echo "    -rp, Release parallel"
  # echo "    -h, --help show this text"
  # echo
}

while [ $# -gt 0 ]; do
  case $1 in
  -c)
    colcon build --packages-select cartographer_ros
    ;;

  -r)
    colcon build
    ;;

  *)
    print_usage
    exit 1
    ;;

  esac

  shift
done

echo "colcon build all packages success!"
