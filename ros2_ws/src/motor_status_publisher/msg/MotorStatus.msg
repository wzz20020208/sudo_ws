# 电机实时运行状态（0x9C 读取，100Hz 发布）
# 力矩用转矩电流 iq 表示：X2-7 无 N·m 直出，iq 与输出力矩成正比

std_msgs/Header header    # 发布时刻（stamp）

float64 angle_deg         # 输出轴角度（°）
float64 speed_dps         # 输出轴转速（°/s）
float64 iq_a              # 转矩电流（A）
