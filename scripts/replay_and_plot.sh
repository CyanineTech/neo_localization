#!/bin/bash
# 此脚本用于从rosbag回放定位数据并在rqt_plot中可视化

# 检查参数
if [ $# -ne 1 ]; then
    echo "使用方法: $0 <rosbag文件路径>"
    exit 1
fi

ROSBAG_FILE=$1

# 检查文件是否存在
if [ ! -f "$ROSBAG_FILE" ]; then
    echo "错误: 找不到rosbag文件 $ROSBAG_FILE"
    exit 1
fi

# 启动rqt_plot
/home/st1g/catkin_ws/src/cyanine-os/neo_localization/scripts/plot_localization_stats.sh &

# 等待rqt_plot启动
sleep 2

# 回放rosbag
echo "开始回放 $ROSBAG_FILE..."
rosbag play $ROSBAG_FILE
