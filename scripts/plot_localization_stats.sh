#!/bin/bash
# 此脚本用于启动rqt_plot并显示LocalizationStats消息的数据

# 可视化定位分数
rqt_plot /neo_localization/stats/score &

# 可视化梯度信息
rqt_plot /neo_localization/stats/grad_uvw[0] /neo_localization/stats/grad_uvw[1] /neo_localization/stats/grad_uvw[2] &

# 可视化标准差
rqt_plot /neo_localization/stats/std_xy /neo_localization/stats/std_yaw &
