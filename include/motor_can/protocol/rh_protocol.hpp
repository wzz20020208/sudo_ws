// include/motor_can/protocol/rh_protocol.hpp
// RH 协议层：单电机 CAN 指令的编解码与单位换算。
//
// 职责边界：
//  - 只回答「某个指令对应的帧长什么样」（编解码 + 物理单位换算），
//    不做收发——发送 / 收回复由调用方用 CanComm 完成（Device 层再组合两者）。
//  - 全部为单电机指令：指令 0x140+ID，回复 0x240+ID，标准帧，DLC=8。
//
// 协议来源：《伺服电机控制协议 V4.3》（适用驱动 V3），与 X2-7 关节模组实测兼容。
// 回复布局：所有控制命令（0xA1/0xA2/0xA4/0xA8/0xA9）的回复与 0x9C 状态2 完全相同，
// 故 decode_run_status 一个函数通吃。

#pragma once

#include "motor_can/can_comm/can_types.hpp"

#include <cstdint>

namespace motor_can {

// 命令字节，与《伺服电机控制协议 V4.3》一一对应。
enum class RhCmd : uint8_t {
    // 参数读写
    PidRead = 0x30,          // 读 PID（回复 float32）
    PidWriteRam = 0x31,      // 写 PID 到 RAM（掉电不保存）
    PidWriteRom = 0x32,      // 写 PID 到 ROM（掉电保存）
    AccelRead = 0x42,        // 读加速度（回复 int32，1°/s²/LSB）
    AccelWrite = 0x43,       // 写加速度到 RAM+ROM（uint32，1°/s²/LSB）
    // 编码器
    EncoderPos = 0x60,       // 读多圈编码器位置（回复 int32，脉冲）
    EncoderRaw = 0x61,       // 读编码器原始位置（回复 int32，脉冲）
    EncoderOffset = 0x62,    // 读编码器零偏（回复 int32，脉冲）
    WriteEncoderOffset = 0x63, // 写编码器多圈零偏到 ROM（int32，脉冲）
    WriteCurrentZero = 0x64, // 当前编码器位置写为零点（需 0x76 复位生效）
    ReadSingleEncoder = 0x90, // 读单圈编码器 encoder/raw/offset（直驱用，回复 3×uint16）
    // 系统
    FunctionControl = 0x20, // 功能控制指令（0x20），索引见 FunctionIndex
    ReadMode = 0x70,        // 读运行模式（回复 Data[7]，见 RunMode）
    SystemReset = 0x76,     // 系统复位（无回复）
    BrakeRelease = 0x77,    // 抱闸释放
    BrakeLock = 0x78,       // 抱闸锁死
    MotorOff = 0x80,        // 关闭电机输出，清除运行状态
    MotorStop = 0x81,        // 停止电机并保持不动
    ReadRunTime = 0xB1,      // 读系统运行时间（uint32，ms）
    ReadVersionDate = 0xB2,  // 读软件版本日期（uint32，yyyymmdd）
    SetComProtectTime = 0xB3, // 通讯中断保护时间设置（uint32 ms，0=关闭，写 ROM）
    SetBaudrate = 0xB4,      // 通讯波特率设置（Data[7]=0/1/2，改完立即断联）
    ReadMotorModel = 0xB5,   // 读电机型号（回复 7 个 ASCII）
    ActiveReport = 0xB6,     // 主动回复设置：指定指令按固定间隔主动上报（无回复）
    // 读状态
    ReadAngle = 0x92,        // 读多圈绝对角度（int32，0.01°/LSB）
    ReadSingleAngle = 0x94,  // 读单圈角度（uint16，0.01°/LSB，直驱电机用）
    ReadStatus = 0x9A,       // 读状态1：温度/MOS温度/抱闸/电压/错误标志
    ReadStatus2 = 0x9C,      // 读状态2：温度/转矩电流/转速/角度
    ReadStatus3 = 0x9D,      // 读状态3：温度 + 三相相电流 iA/iB/iC
    // 控制
    Torque = 0xA1,           // 转矩闭环控制（int16，0.01A/LSB）
    Speed = 0xA2,            // 速度闭环控制（int32，0.01°/s/LSB）
    Position = 0xA4,         // 绝对位置闭环控制（int32，0.01°/LSB）
    SingleAnglePos = 0xA6,   // 单圈位置控制（直驱用，回复同 0x9C）
    IncrementPos = 0xA8,     // 增量位置闭环（int32，0.01°/LSB）
    ForcePos = 0xA9,         // 力控位置闭环（int32，0.01°/LSB）
};

// PID 参数索引（0x30/0x31/0x32 的 Data[1]）
enum class PidIndex : uint8_t {
    CurrentKp = 0x01,  // 电流环 KP
    CurrentKi = 0x02,  // 电流环 KI
    SpeedKp = 0x04,    // 速度环 KP
    SpeedKi = 0x05,    // 速度环 KI
    PositionKp = 0x07, // 位置环 KP
    PositionKi = 0x08, // 位置环 KI
    PositionKd = 0x09, // 位置环 KD
};

// 加速度索引（0x42/0x43 的 Data[1]）
enum class AccelIndex : uint8_t {
    PositionAccel = 0x00, // 位置规划加速度
    PositionDecel = 0x01, // 位置规划减速度
    SpeedAccel = 0x02,    // 速度规划加速度
    SpeedDecel = 0x03,    // 速度规划减速度
};

// 电机运行模式（0x70 回复 Data[7]）
enum class RunMode : uint8_t {
    Current = 0x01,  // 电流环模式
    Speed = 0x02,    // 速度环模式
    Position = 0x03, // 位置环模式
};

// 0x20 功能控制指令的功能索引（Data[1]），Value 放 Data[4..7]（uint32 LE），回复为原帧回显。
// 0x06/0x07 的 Value 单位手册未注明，本层按 0.01°/LSB 打包，Device 层注释标注待真机确认。
enum class FunctionIndex : uint8_t {
    ClearMultiTurn = 0x01,    // 清除多圈值：清多圈、更新零点并保存，重启后生效
    CanIdFilter = 0x02,       // CANID 滤波器使能：1=使能，0=失能（0x280 多电机前置需失能），存 FLASH
    ErrorReport = 0x03,       // 错误状态发送使能：1=出错主动发 0x9A（100ms 周期），0=失能
    MultiTurnPowerSave = 0x04, // 多圈值掉电保存使能：1=掉电保存多圈值，0=单圈模式，重启生效
    SetCanId = 0x05,          // 设置 CANID：Value=新 ID，保存到 ROM，重启后持久化
    MaxPosAngle = 0x06,       // 设置位置运行最大正角度：Value=角度（0.01°/LSB），存 ROM 立即生效
    MaxNegAngle = 0x07,       // 设置位置运行最大负角度：Value=角度（0.01°/LSB），存 ROM 立即生效
};

// 0x9A 状态1 解码结果（已换算成物理单位）
struct MotorStatus {
    int8_t temp_c;        // 电机温度（℃）
    int8_t mos_temp_c;    // MOS 温度（℃）
    bool brake_released;  // true=抱闸已释放
    double voltage_v;     // 供电电压（V）
    uint16_t error_state; // 错误标志，位定义见 System_errorState 表
};

// 0x9C 状态2 及所有控制命令回复的解码结果（已换算成物理单位）
struct MotorRunStatus {
    int8_t temp_c;    // 电机温度（℃）
    double iq_a;      // 转矩电流（A）
    double speed_dps; // 输出轴转速（°/s）
    double angle_deg; // 输出轴角度（°）
};

// 0x9D 状态3 解码结果（已换算成物理单位）
struct MotorStatus3 {
    int8_t temp_c; // 电机温度（℃）
    double ia_a;   // A 相电流（A）
    double ib_a;   // B 相电流（A）
    double ic_a;   // C 相电流（A）
};

// 0x90 单圈编码器解码结果（直驱电机，单位脉冲，uint16）
struct SingleEncoder {
    uint16_t encoder; // 编码器位置 = 原始位置 - 零偏
    uint16_t raw;     // 编码器原始位置
    uint16_t offset;  // 编码器零偏（该点作为角度零点）
};

// ---- 发方向：组帧 ----

// 组一帧单字节命令（抱闸/停止/关闭/复位，以及 0x60~0x70/0x92/0x94/0x9A/0x9C 等只读查询）。
// 命令字节放 Data[0]，其余为 0。
// id 须为 1~32，不做校验。
CanFrame encode_command(uint8_t id, RhCmd cmd);

// 组一帧速度闭环指令（0xA2）。
// speed_dps：目标转速（°/s），按 0.01°/s/LSB 打包，钳位到 int32 范围。
// max_torque_pct：最大扭矩 = 额定电流的百分比（0~255；0=不开启力控）。
CanFrame encode_speed(uint8_t id, double speed_dps, uint8_t max_torque_pct);

// 组一帧转矩闭环指令（0xA1）。current_a：目标电流（A），钳位到 int16 范围。
// 注意：抱闸款电机使用前必须先 0x77 开闸。
CanFrame encode_torque(uint8_t id, double current_a);

// 组一帧绝对位置闭环指令（0xA4）。
// angle_deg：目标角度（°），多圈，按 0.01°/LSB 打包，钳位到 int32 范围。
// max_speed_dps：位置运行最大转速（°/s，1°/s/LSB）。
CanFrame encode_position(uint8_t id, double angle_deg, uint16_t max_speed_dps);

// 组一帧读 PID 指令（0x30）。index：环类型（见 PidIndex），放 Data[1]。
CanFrame encode_read_pid(uint8_t id, PidIndex index);

// 组一帧写 PID 到 RAM 指令（0x31，掉电不保存）。
// value：float32 打包到 Data[4..7]（IEEE754 小端），超 float 精度时按 float 截断。
CanFrame encode_write_pid_ram(uint8_t id, PidIndex index, double value);

// 组一帧写 PID 到 ROM 指令（0x32，掉电保存）。value 同 encode_write_pid_ram。
CanFrame encode_write_pid_rom(uint8_t id, PidIndex index, double value);

// 组一帧读加速度指令（0x42）。index：见 AccelIndex，放 Data[1]。
CanFrame encode_read_accel(uint8_t id, AccelIndex index);

// 组一帧写加速度指令（0x43，写入 RAM+ROM）。
// accel：目标加速度（°/s²），uint32 打包到 Data[4..7]（1°/s²/LSB），钳位到协议范围 100~60000。
CanFrame encode_write_accel(uint8_t id, AccelIndex index, double accel);

// 组一帧写编码器多圈零偏指令（0x63，写入 ROM）。
// offset：零偏脉冲数（int32），打包到 Data[4..7]。
CanFrame encode_write_encoder_offset(uint8_t id, int32_t offset);

// 组一帧增量位置闭环指令（0xA8），布局与 0xA4 相同。
// delta_deg：相对当前位置的角度增量（°），按 0.01°/LSB 打包，钳位到 int32 范围。
// max_speed_dps：位置运行最大转速（°/s，1°/s/LSB）。
CanFrame encode_increment_position(uint8_t id, double delta_deg, uint16_t max_speed_dps);

// 组一帧力控位置闭环指令（0xA9），布局同 0xA4，另带最大扭矩。
// angle_deg：目标多圈角度（°），0.01°/LSB；max_speed_dps：最大转速（°/s，1°/s/LSB）。
// max_torque_pct：最大扭矩 = 额定电流百分比（0~255；0=不开启力控）。
CanFrame encode_force_position(uint8_t id, double angle_deg, uint16_t max_speed_dps,
                               uint8_t max_torque_pct);

// 组一帧设置 CAN ID 指令（0x20 功能控制，索引 0x05）。按地址发到 0x140+current_id，
// 只影响该当前 ID 对应的电机（若多台同 ID 则一起被改）。
// new_id：新 CAN ID（1~32），放 Data[4..7]（uint32 LE）；写入回复为原帧回显。
CanFrame encode_set_can_id(uint8_t current_id, uint8_t new_id);

// 组一帧清除多圈值指令（0x20 功能控制，索引 0x01）。Data[2..7]=0，回复为原帧回显。
// 清除电机多圈值、更新零点并保存；注意手册注明「重启后生效」。
CanFrame encode_clear_multi_turn(uint8_t id);

// 组一帧主动回复设置指令（0xB6，无回复）。
// report_cmd：要主动上报的指令（0x60/0x61/0x62/0x92/0x9A/0x9C/0x9D/0x9E），放 Data[1]。
// enable：true=使能，false=关闭，放 Data[2]。interval_10ms：上报间隔，10ms/LSB，放 Data[3..4]。
// 多条指令设同一间隔时循环交替回复；使能后该指令被电机主动上报、不再回复命令。
CanFrame encode_active_report(uint8_t id, uint8_t report_cmd, bool enable, uint16_t interval_10ms);

// 组一帧 0x20 功能控制指令：Data[1]=index，Data[4..7]=value（uint32 LE），回复为原帧回显。
// 各索引的语义见 FunctionIndex（0x01~0x07）；0x06/0x07 的 value 按 0.01°/LSB 打包。
CanFrame encode_function_control(uint8_t id, FunctionIndex index, uint32_t value);

// 组一帧通讯中断保护时间指令（0xB3）：Data[4..7]=保护时间 uint32（ms，0=关闭），写 ROM。
// 通讯中断超时超过设定时间则切断输出并锁死抱闸；回复为原帧回显。
CanFrame encode_com_protect_time(uint8_t id, uint32_t time_ms);

// 组一帧通讯波特率设置指令（0xB4）：Data[7]=baudrate（0=RS485 115200/CAN 500k，
// 1=RS485 500k/CAN 1M，2=RS485 1M/CAN 无效）。改完立即断联，回复随机内容无需处理。
// ⚠️ 本工程固定 1Mbps，Device 层不暴露此指令。
CanFrame encode_set_baudrate(uint8_t id, uint8_t baudrate);

// 组一帧单圈位置控制指令（0xA6，直驱用）。
// direction：0=顺时针，1=逆时针，放 Data[1]；max_speed_dps：最大转速（°/s，1°/LSB），
// Data[2..3]；angle_deg：目标单圈角度（0°~359.99°），按 0.01°/LSB 打包 uint16 放 Data[4..5]。
// 回复布局同 0x9C（末字段为单圈编码器值而非角度）。
CanFrame encode_single_angle_position(uint8_t id, uint8_t direction, uint16_t max_speed_dps,
                                      double angle_deg);

// 把任意单电机指令帧转为 0x280 广播帧（仅 ID 改为 0x280，数据不变）：总线上所有电机
// 同时响应，各自在 0x240+ID 回复。前置：需先失能 0x20 索引 0x02 CANID 滤波器。
CanFrame to_broadcast(const CanFrame& single_frame);

// ---- 收方向：解帧 ----

// 解码 0x9A 状态1 回复。命令字节不符或 DLC 不足时返回 false。
bool decode_status(const CanFrame& reply, MotorStatus& out);

// 解码 0x9C 状态2，或任一控制命令（0xA1/0xA2/0xA4/0xA8/0xA9）的回复（布局一致）。
// 命令字节不符或 DLC 不足时返回 false。
bool decode_run_status(const CanFrame& reply, MotorRunStatus& out);

// 解码 0x92 多圈角度回复，输出角度（°）。命令字节不符或 DLC 不足时返回 false。
bool decode_angle(const CanFrame& reply, double& angle_deg);

// 解码 PID 回复（0x30/0x31/0x32，float32 位域还原为 float）。
// value：该环 PID 参数。命令字节不符或 DLC 不足时返回 false。
bool decode_pid(const CanFrame& reply, float& value);

// 解码加速度回复（0x42/0x43）。value：加速度（°/s²，int32，1°/s²/LSB）。
// 命令字节不符或 DLC 不足时返回 false。
bool decode_accel(const CanFrame& reply, double& value);

// 解码编码器位置回复（0x60/0x61/0x62/0x63/0x64，int32@Data[4..7]）。
// pos：编码器脉冲数（0x60 多圈位置 / 0x61 原始位置 / 0x62 零偏 /
// 0x63 写零偏回显 / 0x64 零点写回显的新零偏）。命令字节不符或 DLC 不足时返回 false。
bool decode_encoder_position(const CanFrame& reply, int32_t& pos);

// 解码 0x94 单圈角度回复。angle_deg：角度（°），uint16@Data[6..7]，0.01°/LSB。
// 命令字节不符或 DLC 不足时返回 false。
bool decode_single_angle(const CanFrame& reply, double& angle_deg);

// 解码 0x70 运行模式回复。mode：见 RunMode（Data[7]）。
// 命令字节不符、DLC 不足或模式值非法时返回 false。
bool decode_mode(const CanFrame& reply, RunMode& mode);

// 解码 0x20 索引 0x05 设置 CANID 回显。id：回显的 CANID（Data[4..7] uint32 LE，1~32）。
// 命令字节、功能索引不符或 DLC 不足时返回 false。
bool decode_set_can_id(const CanFrame& reply, uint8_t& id);

// 校验 0x20 索引 0x01 清除多圈值回显（原帧回显，无其他数据，命令字节 + 功能索引
// 均匹配才算成功）。命令字节、功能索引不符或 DLC 不足时返回 false。
bool decode_clear_multi_turn(const CanFrame& reply);

// 校验 0x20 功能控制指令回显：命令字节为 0x20、索引为 index、DLC 足够时返回 true，
// 并回读 Data[4..7]（uint32 LE）到 value。
bool decode_function_control_echo(const CanFrame& reply, FunctionIndex index, uint32_t& value);

// 解码 0x9D 状态3 回复。out：温度（℃）+ 三相相电流（A）。命令字节不符或 DLC 不足返回 false。
bool decode_status3(const CanFrame& reply, MotorStatus3& out);

// 解码 0x90 单圈编码器回复。enc：encoder/raw/offset（脉冲，uint16 小端，Data[2..7]）。
// 命令字节不符或 DLC 不足时返回 false。
bool decode_single_encoder(const CanFrame& reply, SingleEncoder& enc);

// 解码 0xB1 系统运行时间回复。time_ms：自复位起的运行时间（uint32 LE，Data[4..7]，ms）。
// 命令字节不符或 DLC 不足时返回 false。
bool decode_run_time(const CanFrame& reply, uint32_t& time_ms);

// 解码 0xB2 软件版本日期回复。date：yyyymmdd（uint32 LE，Data[4..7]，如 0x0134892E → 20220206）。
// 命令字节不符或 DLC 不足时返回 false。
bool decode_version_date(const CanFrame& reply, uint32_t& date);

// 解码 0xB5 电机型号回复。model：Data[1..7] 的 7 个 ASCII 字符，末尾自动补 '\0'。
// 命令字节不符或 DLC 不足时返回 false。
bool decode_motor_model(const CanFrame& reply, char model[8]);

}  // namespace motor_can
