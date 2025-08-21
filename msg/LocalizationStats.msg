Header header
float32 score
float32[3] grad_uvw
float32 std_xy
float32 std_yaw
int32 mode
# 异常检测结果（在filtered话题中特别有意义）
bool abnormal
# 级别：1=INFO(正常), 2=WARN(预警), 3=ERROR(错误)
uint8 level
string message
# 风险（主要在filtered话题有效）
float32 risk