Header header
float32 score
float32[3] grad_uvw
float32 std_xy
float32 std_yaw
int32 mode
# 新增：异常检测结果（在filtered话题中特别有意义）
bool abnormal
uint8 level  # 1=INFO, 2=WARN, 3=ERROR
string message