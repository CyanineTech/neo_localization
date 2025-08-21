# neo_localization数据记录与可视化指南

本文档介绍如何记录`neo_localization`节点发布的`LocalizationStats`消息，并使用rqt_plot进行可视化。

## 话题速览

- 原始统计：`/neo_localization_node/localization_stats`
- 滤波统计：`/neo_localization_node/localization_stats_filtered`
  - 新增字段：
    - `abnormal`：是否异常（level>=2 为true）
    - `level`：1=INFO(正常), 2=WARN(预警), 3=ERROR(错误)
    - `message`：人类可读的预警/错误信息
    - `risk`：连续、平滑的定位风险值[0..1]

## 数据记录

1) 启动定位节点（示例测试环境）：

```bash
roslaunch neo_localization test_setup.launch
```

2) 另启一个终端，启动记录功能（仅启动rosbag记录，不会启动节点）：

```bash
roslaunch neo_localization record_localization_stats.launch
```

这将把上述两个话题(`localization_stats`与`localization_stats_filtered`)记录到`neo_localization/bags/`目录下的rosbag文件。

### 停止记录

数据记录完成后，在运行rosbag的终端按`Ctrl+C`停止记录。记录的数据保存在`/home/st1g/catkin_ws/src/cyanine-os/neo_localization/bags/`目录下，文件名格式为`localization_stats_<robot>_<map>_YYYY-MM-DD-HH-MM-SS.bag`。

## 数据可视化

### 实时数据可视化

在机器人运行过程中，可以实时查看风险与等级：

```bash
rqt_plot /neo_localization_node/localization_stats_filtered/risk
rqt_plot /neo_localization_node/localization_stats_filtered/level
rqt_plot /neo_localization_node/localization_stats_filtered/abnormal
```

也可查看基础指标（来自原始/滤波后消息的相同字段）：
- `score`
- `grad_uvw[0]`, `grad_uvw[1]`, `grad_uvw[2]`
- `std_xy`
- `std_yaw`
- `mode`

例如：

```bash
rqt_plot /neo_localization_node/localization_stats_filtered/score
```

### 回放记录的数据

```bash
rosbag play /path/to/your/bagfile.bag
```
然后用`rqt_plot`绘制上述话题。

## 风险评估参数（可在`launch/test_setup.yaml`中配置）

- 帧计数：
  - `error_frames`：达到错误阈值累积多少帧判定ERROR
  - `warn_frames`：达到预警阈值累积多少帧判定WARN
  - `clear_frames`：低于清除阈值累积多少帧回到INFO
- 风险阈值与滞回：
  - `risk_err`：错误上阈值
  - `risk_warn`：预警阈值
  - `risk_clear`：清除下阈值
- 平滑与软/硬策略：
  - `risk_alpha`：EWMA平滑系数
  - `soft_margin`：软阈值线性化带宽
  - `hard_bonus`：硬条件附加惩罚
- 指标权重：`w_score`, `w_uvw0`, `w_uvw1`, `w_stdxy`, `w_stdyaw`
- 错误判据（硬条件）：`th_*_err`

调参建议：
- 首先用默认值运行并记录包，观察`risk`是否在预期情况下上升、回落；
- 调整`risk_alpha`与`*_frames`以抑制抖动；
- 调整`risk_warn`/`risk_err`与`risk_clear`间距形成合理滞回带；
- 必要时微调各权重与`soft_margin`，提升渐进性与鲁棒性。

## 注意事项

- 启动/记录命令在不同终端执行；先启动节点，再启动记录。
- rosbag文件可能较大，请确保磁盘空间充足。
- 如需仅记录特定时长，可在`rosbag record`参数中加入`-d <seconds>`。
