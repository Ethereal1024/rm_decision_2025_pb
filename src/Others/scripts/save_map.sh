#!/bin/bash

read -p '按 [Enter] 保存地图...' key

set -e

MAP_DIR="../saved_maps"
mkdir -p "$MAP_DIR"

MAP_NAME="${MAP_DIR}/map_$(date +'%Y%m%d_%H%M%S')"

if ! ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"; then
    echo "错误: 地图保存失败！"
    exit 1
fi

echo "地图已保存至: ${MAP_NAME}.pgm 和 ${MAP_NAME}.yaml"
