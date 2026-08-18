// src/protocol/rh_protocol.cpp
// RH 协议层实现：组帧（发方向）+ 解帧（收方向），全部小端序。

#include "motor_can/protocol/rh_protocol.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>

namespace motor_can {
namespace {

// 组一帧基础指令帧：0x140+ID、标准帧、DLC=8、命令字节放 Data[0]，其余清零
CanFrame make_base(uint8_t id, uint8_t cmd) {
    CanFrame f{};
    f.id = 0x140u + id;
    f.is_extended = false;
    f.dlc = 8;
    f.data[0] = cmd;
    return f;
}

// 小端序写 / 读 16 位值（bit 打包，无符号按位操作，有符号值 bit 相同）
void put_le16(uint8_t* data, uint16_t v) {
    data[0] = static_cast<uint8_t>(v);
    data[1] = static_cast<uint8_t>(v >> 8);
}
void put_le32(uint8_t* data, uint32_t v) {
    data[0] = static_cast<uint8_t>(v);
    data[1] = static_cast<uint8_t>(v >> 8);
    data[2] = static_cast<uint8_t>(v >> 16);
    data[3] = static_cast<uint8_t>(v >> 24);
}
uint16_t get_le16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}
uint32_t get_le32(const uint8_t* data) {
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

// 物理值打包前的钳位：double 换算后超出目标位宽时截到边界
int16_t clamp_i16(double v) {
    return static_cast<int16_t>(std::clamp(v, static_cast<double>(INT16_MIN),
                                              static_cast<double>(INT16_MAX)));
}
int32_t clamp_i32(double v) {
    return static_cast<int32_t>(std::clamp(v, static_cast<double>(INT32_MIN),
                                              static_cast<double>(INT32_MAX)));
}

// double -> float32（IEEE754）的 32 位模式，用于 PID 值打包
uint32_t float_bits(double v) {
    const float f = static_cast<float>(v);
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    return bits;
}

// 小端读 float32（IEEE754），用于 PID 回复解包
float bits_float(const uint8_t* data) {
    float f = 0.0f;
    const uint32_t bits = get_le32(data);
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

}  // namespace

// 单字节命令 / 只读查询：命令字节放 Data[0]，其余全 0。
// 0x77/0x78/0x80/0x81 的回复为原帧回显；0x76 系统复位无回复；
// 0x60~0x70/0x92/0x94/0x9A/0x9C 查询的回复布局见对应 decode_* 函数。
CanFrame encode_command(uint8_t id, RhCmd cmd) {
    return make_base(id, static_cast<uint8_t>(cmd));
}

// 速度闭环 0xA2 指令布局：
//  Data[1]      最大扭矩，单位 = 额定电流的 1%/LSB（0~255；0 = 不开启力控）
//  Data[4..7]   目标转速 int32，单位 = 0.01°/s/LSB（100dps -> 0x00002710）
CanFrame encode_speed(uint8_t id, double speed_dps, uint8_t max_torque_pct) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::Speed));
    f.data[1] = max_torque_pct;
    put_le32(f.data + 4, static_cast<uint32_t>(clamp_i32(speed_dps * 100.0)));
    return f;
}

// 转矩闭环 0xA1 指令布局：
//  Data[4..5]   目标电流 int16，单位 = 0.01A/LSB（1A -> 0x0064）
//  抱闸款电机使用前必须先 0x77 开闸，否则不转。
CanFrame encode_torque(uint8_t id, double current_a) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::Torque));
    put_le16(f.data + 4, static_cast<uint16_t>(clamp_i16(current_a * 100.0)));
    return f;
}

// 绝对位置闭环 0xA4 指令布局：
//  Data[2..3]   位置运行最大转速 uint16，单位 = 1°/s/LSB
//  Data[4..7]   目标角度 int32（多圈），单位 = 0.01°/LSB（360° -> 0x00008CA0）
CanFrame encode_position(uint8_t id, double angle_deg, uint16_t max_speed_dps) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::Position));
    put_le16(f.data + 2, max_speed_dps);
    put_le32(f.data + 4, static_cast<uint32_t>(clamp_i32(angle_deg * 100.0)));
    return f;
}

// 读 PID 0x30 指令布局：
//  Data[1]   环类型索引（见 PidIndex）
CanFrame encode_read_pid(uint8_t id, PidIndex index) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::PidRead));
    f.data[1] = static_cast<uint8_t>(index);
    return f;
}

// 内部：组一帧写 PID 帧（0x31/0x32 共用布局）：
//  Data[1]   环类型索引（见 PidIndex）
//  Data[4..7]   PID 值 float32（IEEE754 小端，1.5f -> 0x3FC00000）
CanFrame pid_write_frame(uint8_t id, uint8_t cmd, PidIndex index, double value) {
    CanFrame f = make_base(id, cmd);
    f.data[1] = static_cast<uint8_t>(index);
    put_le32(f.data + 4, float_bits(value));
    return f;
}

// 写 PID 到 RAM（0x31），掉电不保存
CanFrame encode_write_pid_ram(uint8_t id, PidIndex index, double value) {
    return pid_write_frame(id, static_cast<uint8_t>(RhCmd::PidWriteRam), index, value);
}

// 写 PID 到 ROM（0x32），掉电保存
CanFrame encode_write_pid_rom(uint8_t id, PidIndex index, double value) {
    return pid_write_frame(id, static_cast<uint8_t>(RhCmd::PidWriteRom), index, value);
}

// 读加速度 0x42 指令布局：
//  Data[1]   加速度类型索引（见 AccelIndex）
CanFrame encode_read_accel(uint8_t id, AccelIndex index) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::AccelRead));
    f.data[1] = static_cast<uint8_t>(index);
    return f;
}

// 写加速度 0x43 指令布局（写入 RAM+ROM）：
//  Data[1]   加速度类型索引（见 AccelIndex）
//  Data[4..7]   加速度 uint32（1°/s²/LSB，协议范围 100~60000）
CanFrame encode_write_accel(uint8_t id, AccelIndex index, double accel) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::AccelWrite));
    f.data[1] = static_cast<uint8_t>(index);
    const double clamped = std::clamp(accel, 100.0, 60000.0);
    put_le32(f.data + 4, static_cast<uint32_t>(std::lround(clamped)));
    return f;
}

// 写编码器多圈零偏 0x63 指令布局（写入 ROM）：
//  Data[4..7]   多圈零偏 int32（脉冲）
CanFrame encode_write_encoder_offset(uint8_t id, int32_t offset) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::WriteEncoderOffset));
    put_le32(f.data + 4, static_cast<uint32_t>(offset));
    return f;
}

// 增量位置闭环 0xA8 指令布局（与 0xA4 相同）：
//  Data[2..3]   位置运行最大转速 uint16（1°/s/LSB）
//  Data[4..7]   相对当前位置的角度增量 int32（0.01°/LSB，360° -> 0x00008CA0）
CanFrame encode_increment_position(uint8_t id, double delta_deg, uint16_t max_speed_dps) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::IncrementPos));
    put_le16(f.data + 2, max_speed_dps);
    put_le32(f.data + 4, static_cast<uint32_t>(clamp_i32(delta_deg * 100.0)));
    return f;
}

// 力控位置闭环 0xA9 指令布局：
//  Data[1]      最大扭矩 = 额定电流百分比（0~255；0 = 不开启力控）
//  Data[2..3]   位置运行最大转速 uint16（1°/s/LSB）
//  Data[4..7]   目标多圈角度 int32（0.01°/LSB）
CanFrame encode_force_position(uint8_t id, double angle_deg, uint16_t max_speed_dps,
                               uint8_t max_torque_pct) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::ForcePos));
    f.data[1] = max_torque_pct;
    put_le16(f.data + 2, max_speed_dps);
    put_le32(f.data + 4, static_cast<uint32_t>(clamp_i32(angle_deg * 100.0)));
    return f;
}

// 0x20 功能控制指令通用布局（索引见 FunctionIndex）：
//  Data[1]      功能索引
//  Data[2..3]   保留
//  Data[4..7]   输入参数 Value uint32（小端），各索引语义见 FunctionIndex 注释
// 回复为原帧回显（Data[4..7] 原样带回）
CanFrame encode_function_control(uint8_t id, FunctionIndex index, uint32_t value) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::FunctionControl));
    f.data[1] = static_cast<uint8_t>(index);
    put_le32(f.data + 4, value);
    return f;
}

// 0x20 索引 0x05 设置 CANID，按地址发到 0x140+current_id：
// 只影响该当前 ID 对应的电机（若多台同 ID 则一起被改）
CanFrame encode_set_can_id(uint8_t current_id, uint8_t new_id) {
    return encode_function_control(current_id, FunctionIndex::SetCanId, new_id);
}

// 0x20 索引 0x01 清除多圈值：Value=0，回复为原帧回显
CanFrame encode_clear_multi_turn(uint8_t id) {
    return encode_function_control(id, FunctionIndex::ClearMultiTurn, 0);
}

// 0xB6 主动回复设置指令布局：
//  Data[1]      要主动上报的指令（0x60/0x61/0x62/0x92/0x9A/0x9C/0x9D/0x9E）
//  Data[2]      使能位（1=使能，0=关闭）
//  Data[3..4]   上报间隔 uint16（10ms/LSB）
//  无回复；使能后电机按固定间隔主动上报该指令，不再回复命令
CanFrame encode_active_report(uint8_t id, uint8_t report_cmd, bool enable,
                              uint16_t interval_10ms) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::ActiveReport));
    f.data[1] = report_cmd;
    f.data[2] = enable ? 1 : 0;
    put_le16(f.data + 3, interval_10ms);
    return f;
}

// 0xB3 通讯中断保护时间指令布局：
//  Data[4..7]   保护时间 CanRecvTime_MS uint32（ms，0=不使能），写 ROM 掉电保存
CanFrame encode_com_protect_time(uint8_t id, uint32_t time_ms) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::SetComProtectTime));
    put_le32(f.data + 4, time_ms);
    return f;
}

// 0xB4 通讯波特率设置指令布局：
//  Data[7]      baudrate：0=RS485 115200/CAN 500k，1=RS485 500k/CAN 1M，2=RS485 1M/CAN 无效
//  修改后立即按新波特率运行，回复内容随机无需处理
CanFrame encode_set_baudrate(uint8_t id, uint8_t baudrate) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::SetBaudrate));
    f.data[7] = baudrate;
    return f;
}

// 0xA6 单圈位置控制指令布局：
//  Data[1]      转动方向（0=顺时针，1=逆时针）
//  Data[2..3]   最大转速 maxSpeed uint16（1dps/LSB）
//  Data[4..5]   目标角度 angleControl uint16（0.01°/LSB）
CanFrame encode_single_angle_position(uint8_t id, uint8_t direction, uint16_t max_speed_dps,
                                      double angle_deg) {
    CanFrame f = make_base(id, static_cast<uint8_t>(RhCmd::SingleAnglePos));
    f.data[1] = direction;
    put_le16(f.data + 2, max_speed_dps);
    const long raw = std::lround(angle_deg * 100.0);
    // 手册示例中 360° 打包为 36000（0x8CA0，单圈里与 0° 重合），该值显式放行；
    // 其余超出 0~35999 的输入钳位到边界（36000 以外的非法值）
    const long packed = raw == 36000 ? 36000 : std::clamp(raw, 0L, 35999L);
    put_le16(f.data + 4, static_cast<uint16_t>(packed));
    return f;
}

// 0x280 广播帧：仅把 ID 改为 0x280，数据域不变。总线上所有电机同时响应。
CanFrame to_broadcast(const CanFrame& single_frame) {
    CanFrame f = single_frame;
    f.id = 0x280u;
    return f;
}

// 0x9A 状态1 回复布局：
//  Data[1]      电机温度 int8（℃）
//  Data[2]      MOS 温度 int8（℃）
//  Data[3]      抱闸状态（1 = 已释放）
//  Data[4..5]   供电电压 uint16（0.1V/LSB）
//  Data[6..7]   错误标志 uint16（System_errorState 位定义，可叠加）
bool decode_status(const CanFrame& reply, MotorStatus& out) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadStatus)) {
        return false;
    }
    out.temp_c = static_cast<int8_t>(reply.data[1]);
    out.mos_temp_c = static_cast<int8_t>(reply.data[2]);
    out.brake_released = reply.data[3] != 0;
    out.voltage_v = get_le16(reply.data + 4) * 0.1;
    out.error_state = get_le16(reply.data + 6);
    return true;
}

// 0x9C 状态2 与所有控制命令（0xA1/0xA2/0xA4/0xA6/0xA8/0xA9）回复布局一致：
//  Data[1]      电机温度 int8（℃）
//  Data[2..3]   转矩电流 iq int16（0.01A/LSB）
//  Data[4..5]   输出轴转速 int16（1°/s/LSB）
//  Data[6..7]   输出轴角度 int16（1°/LSB，范围 ±32767°）；
//               0xA6 单圈位置回复末字段为单圈编码器值（脉冲），非角度
bool decode_run_status(const CanFrame& reply, MotorRunStatus& out) {
    if (reply.dlc < 8) {
        return false;
    }
    const uint8_t cmd = reply.data[0];
    // 0x9C 状态2 与所有控制命令的回复布局一致
    if (cmd != static_cast<uint8_t>(RhCmd::ReadStatus2) &&
        cmd != static_cast<uint8_t>(RhCmd::Torque) &&
        cmd != static_cast<uint8_t>(RhCmd::Speed) &&
        cmd != static_cast<uint8_t>(RhCmd::Position) &&
        cmd != static_cast<uint8_t>(RhCmd::SingleAnglePos) &&
        cmd != static_cast<uint8_t>(RhCmd::IncrementPos) &&
        cmd != static_cast<uint8_t>(RhCmd::ForcePos)) {
        return false;
    }
    out.temp_c = static_cast<int8_t>(reply.data[1]);
    out.iq_a = static_cast<int16_t>(get_le16(reply.data + 2)) * 0.01;
    out.speed_dps = static_cast<int16_t>(get_le16(reply.data + 4));  // 1°/s/LSB
    out.angle_deg = static_cast<int16_t>(get_le16(reply.data + 6));  // 1°/LSB
    return true;
}

// 0x92 多圈绝对角度回复：Data[4..7] 角度 int32（0.01°/LSB，输出轴相对零点的多圈角度）
bool decode_angle(const CanFrame& reply, double& angle_deg) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadAngle)) {
        return false;
    }
    angle_deg = static_cast<int32_t>(get_le32(reply.data + 4)) * 0.01;
    return true;
}

// PID 回复（0x30/0x31/0x32 布局相同）：Data[4..7] 为 PID 值 float32（IEEE754 小端）
bool decode_pid(const CanFrame& reply, float& value) {
    if (reply.dlc < 8) {
        return false;
    }
    const uint8_t cmd = reply.data[0];
    if (cmd != static_cast<uint8_t>(RhCmd::PidRead) &&
        cmd != static_cast<uint8_t>(RhCmd::PidWriteRam) &&
        cmd != static_cast<uint8_t>(RhCmd::PidWriteRom)) {
        return false;
    }
    value = bits_float(reply.data + 4);
    return true;
}

// 加速度回复（0x42/0x43 布局相同）：Data[4..7] 为加速度 int32（1°/s²/LSB）
bool decode_accel(const CanFrame& reply, double& value) {
    if (reply.dlc < 8) {
        return false;
    }
    const uint8_t cmd = reply.data[0];
    if (cmd != static_cast<uint8_t>(RhCmd::AccelRead) &&
        cmd != static_cast<uint8_t>(RhCmd::AccelWrite)) {
        return false;
    }
    value = static_cast<int32_t>(get_le32(reply.data + 4));
    return true;
}

// 编码器位置回复（0x60/0x61/0x62/0x63/0x64 布局相同）：Data[4..7] 为位置 int32（脉冲）
// 0x60 多圈位置 / 0x61 原始位置 / 0x62 零偏 / 0x63 写零偏回显 / 0x64 零点写回显，
// 同一解码器通吃（与 decode_pid 接受 0x31/0x32 写回显一致）。
bool decode_encoder_position(const CanFrame& reply, int32_t& pos) {
    if (reply.dlc < 8) {
        return false;
    }
    const uint8_t cmd = reply.data[0];
    if (cmd != static_cast<uint8_t>(RhCmd::EncoderPos) &&
        cmd != static_cast<uint8_t>(RhCmd::EncoderRaw) &&
        cmd != static_cast<uint8_t>(RhCmd::EncoderOffset) &&
        cmd != static_cast<uint8_t>(RhCmd::WriteEncoderOffset) &&
        cmd != static_cast<uint8_t>(RhCmd::WriteCurrentZero)) {
        return false;
    }
    pos = static_cast<int32_t>(get_le32(reply.data + 4));
    return true;
}

// 0x94 单圈角度回复布局：
//  Data[6..7]   单圈角度 uint16（0.01°/LSB，直驱电机使用）
bool decode_single_angle(const CanFrame& reply, double& angle_deg) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadSingleAngle)) {
        return false;
    }
    angle_deg = get_le16(reply.data + 6) * 0.01;
    return true;
}

// 0x70 运行模式回复布局：
//  Data[7]   运行模式（1=电流环 2=速度环 3=位置环），非法值返回 false
bool decode_mode(const CanFrame& reply, RunMode& mode) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadMode)) {
        return false;
    }
    const auto m = static_cast<RunMode>(reply.data[7]);
    if (m != RunMode::Current && m != RunMode::Speed && m != RunMode::Position) {
        return false;
    }
    mode = m;
    return true;
}

// 0x20 功能控制指令回显（原帧回显）：命令字节 + 功能索引一致即接受，回读 Value。
// 各索引的 Value 语义见 FunctionIndex 注释。
bool decode_function_control_echo(const CanFrame& reply, FunctionIndex index, uint32_t& value) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::FunctionControl) ||
        reply.data[1] != static_cast<uint8_t>(index)) {
        return false;
    }
    value = get_le32(reply.data + 4);
    return true;
}

// 0x20 索引 0x05 设置 CANID 回显：Value 为新 ID（1~32）
bool decode_set_can_id(const CanFrame& reply, uint8_t& id) {
    uint32_t value = 0;
    if (!decode_function_control_echo(reply, FunctionIndex::SetCanId, value)) {
        return false;
    }
    if (value < 1 || value > 32) {
        return false;
    }
    id = static_cast<uint8_t>(value);
    return true;
}

// 0x20 索引 0x01 清除多圈值回显：仅需命令字节 + 索引一致，Value 恒为 0
bool decode_clear_multi_turn(const CanFrame& reply) {
    uint32_t value = 0;
    return decode_function_control_echo(reply, FunctionIndex::ClearMultiTurn, value);
}

// 0x9D 状态3 回复布局：
//  Data[1]      电机温度 int8（℃）
//  Data[2..3]   A 相电流 int16（0.01A/LSB）
//  Data[4..5]   B 相电流 int16（0.01A/LSB）
//  Data[6..7]   C 相电流 int16（0.01A/LSB）
bool decode_status3(const CanFrame& reply, MotorStatus3& out) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadStatus3)) {
        return false;
    }
    out.temp_c = static_cast<int8_t>(reply.data[1]);
    out.ia_a = static_cast<int16_t>(get_le16(reply.data + 2)) * 0.01;
    out.ib_a = static_cast<int16_t>(get_le16(reply.data + 4)) * 0.01;
    out.ic_a = static_cast<int16_t>(get_le16(reply.data + 6)) * 0.01;
    return true;
}

// 0x90 单圈编码器回复布局（手册数据域表格行号有笔误，按通讯示例确定）：
//  Data[2..3]   编码器位置 encoder uint16（原始位置 - 零偏）
//  Data[4..5]   编码器原始位置 encoderRaw uint16
//  Data[6..7]   编码器零偏 encoderOffset uint16
bool decode_single_encoder(const CanFrame& reply, SingleEncoder& enc) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadSingleEncoder)) {
        return false;
    }
    enc.encoder = get_le16(reply.data + 2);
    enc.raw = get_le16(reply.data + 4);
    enc.offset = get_le16(reply.data + 6);
    return true;
}

// 0xB1 系统运行时间回复：Data[4..7] = SysRunTime uint32（小端，ms）
bool decode_run_time(const CanFrame& reply, uint32_t& time_ms) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadRunTime)) {
        return false;
    }
    time_ms = get_le32(reply.data + 4);
    return true;
}

// 0xB2 软件版本日期回复：Data[4..7] = VersionDate uint32（小端，yyyymmdd）
bool decode_version_date(const CanFrame& reply, uint32_t& date) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadVersionDate)) {
        return false;
    }
    date = get_le32(reply.data + 4);
    return true;
}

// 0xB5 电机型号回复：Data[1..7] = 7 个 ASCII 字符（如 58 38 53 32 56 31 30 → "X8S2V10"）
bool decode_motor_model(const CanFrame& reply, char model[8]) {
    if (reply.dlc < 8 || reply.data[0] != static_cast<uint8_t>(RhCmd::ReadMotorModel)) {
        return false;
    }
    for (int i = 0; i < 7; ++i) {
        model[i] = static_cast<char>(reply.data[i + 1]);
    }
    model[7] = '\0';
    return true;
}

}  // namespace motor_can
