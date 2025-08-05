# neo_localization数据记录与可视化指南

本文档介绍如何记录`neo_localization`节点发布的`LocalizationStats`消息，并使用rqt_plot进行可视化。

## 数据记录

### 步骤1: 记录数据

启动记录功能：

```bash
roslaunch neo_localization record_localization_stats.launch
```

这将启动neo_localization节点并记录`/neo_localization/stats`话题到`neo_localization/bags/`目录下的rosbag文件。

### 步骤2: 停止记录

数据记录完成后，按`Ctrl+C`停止记录过程。记录的数据将保存在`/home/st1g/catkin_ws/src/cyanine-os/neo_localization/bags/`目录下，文件名格式为`localization_stats_YYYY-MM-DD-HH-MM-SS.bag`。

## 数据可视化

### 实时数据可视化

在机器人运行过程中，您可以使用以下命令实时查看数据：

```bash
./scripts/plot_localization_stats.sh
```

这将打开多个rqt_plot窗口，显示各个数据指标的实时变化。

### 回放记录的数据

如果您想回放之前记录的数据，可以使用以下命令：

```bash
./scripts/replay_and_plot.sh /path/to/your/bagfile.bag
```

例如：

```bash
./scripts/replay_and_plot.sh /home/st1g/catkin_ws/src/cyanine-os/neo_localization/bags/localization_stats_2023-10-20-14-30-45.bag
```

## 记录的数据指标

以下是`LocalizationStats`消息中记录的主要数据指标：

1. `score` - 定位分数，表示当前定位的可信度
2. `grad_uvw` - 梯度向量，包含三个分量 [x, y, z]
3. `std_xy` - x和y方向的标准差
4. `std_yaw` - 偏航角的标准差
5. `mode` - 定位模式

## 注意事项

- 确保在使用这些脚本之前，ROS环境已经正确设置。
- rosbag文件可能会随着时间增长变得非常大，请确保有足够的磁盘空间。
- 如果只需要记录特定时间段的数据，可以手动修改launch文件中的参数，例如添加`-d 60`来只记录60秒。
