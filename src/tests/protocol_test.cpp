// tests/protocol_test.cpp
// RH 协议层自测（纯编解码，不碰硬件，无需 CAN 接口）：
//   1. 组帧字段（ID/DLC/数据域）与《伺服电机控制协议 V4.3》示例逐字节比对
//   2. 解帧与手册示例回复比对（温度/电流/转速/角度/电压/抱闸/错误）
//   3. 异常路径：命令字节不符、DLC 不足、超范围钳位
// 用法: ./protocol_test

#include "motor_can/protocol/rh_protocol.hpp"

#include "motor_can/common/log.hpp"

#include <cmath>
#include <cstdint>
#include <string>

namespace {

using motor_can::CanFrame;
using motor_can::RhCmd;

int g_failures = 0;

void expect(bool cond, const char* what) {
    if (!cond) {
        ++g_failures;
        MC_LOG_ERROR("FAIL: %s", what);
    } else {
        MC_LOG_INFO("PASS: %s", what);
    }
}

bool near(double a, double b) {
    return std::fabs(a - b) < 1e-9;
}

bool same_data(const uint8_t a[8], const uint8_t b[8]) {
    for (int i = 0; i < 8; ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

// 组一帧标准回复：命令字节在 Data[0]，payload[7] 填其余字节
CanFrame reply_frame(uint8_t cmd_byte, const uint8_t payload[7]) {
    CanFrame f{};
    f.id = 0x241;
    f.is_extended = false;
    f.dlc = 8;
    f.data[0] = cmd_byte;
    for (int i = 1; i < 8; ++i) {
        f.data[i] = payload[i - 1];
    }
    return f;
}

// ---- 组帧（发方向）----

void test_encode_command() {
    const auto f = motor_can::encode_command(1, RhCmd::BrakeRelease);
    expect(f.id == 0x141 && !f.is_extended && f.dlc == 8, "encode_command: ID=0x141/标准帧/DLC=8");
    const uint8_t want[8] = {0x77, 0, 0, 0, 0, 0, 0, 0};
    expect(same_data(f.data, want), "encode_command(0x77): 命令字节在 Data[0]，其余清零");

    const auto q = motor_can::encode_command(1, RhCmd::ReadStatus);
    expect(q.data[0] == 0x9A, "encode_command(0x9A): 只读查询命令字节");
}

void test_encode_speed() {
    // 手册示例 1：100dps → 10000，Data[1]=50% 限扭
    const auto f = motor_can::encode_speed(1, 100.0, 50);
    const uint8_t want[8] = {0xA2, 50, 0, 0, 0x10, 0x27, 0x00, 0x00};
    expect(same_data(f.data, want), "encode_speed(100dps, 50%): 与手册示例逐字节一致");

    // 手册示例 2：-100dps → 0xFFFFD8F0
    const auto g = motor_can::encode_speed(1, -100.0, 0);
    const uint8_t want_neg[8] = {0xA2, 0, 0, 0, 0xF0, 0xD8, 0xFF, 0xFF};
    expect(same_data(g.data, want_neg), "encode_speed(-100dps): 负数 int32 小端打包");
}

void test_encode_torque() {
    // 手册示例 1：1A → 100
    const auto f = motor_can::encode_torque(1, 1.0);
    const uint8_t want[8] = {0xA1, 0, 0, 0, 0x64, 0x00, 0, 0};
    expect(same_data(f.data, want), "encode_torque(1A): 与手册示例逐字节一致");

    // 手册示例 2：-1A → 0xFF9C
    const auto g = motor_can::encode_torque(1, -1.0);
    const uint8_t want_neg[8] = {0xA1, 0, 0, 0, 0x9C, 0xFF, 0, 0};
    expect(same_data(g.data, want_neg), "encode_torque(-1A): 负数 int16 小端打包");

    // 超范围钳位：1000A → int16 上限 327.67A
    const auto h = motor_can::encode_torque(1, 1000.0);
    const uint8_t want_clamp[8] = {0xA1, 0, 0, 0, 0xFF, 0x7F, 0, 0};
    expect(same_data(h.data, want_clamp), "encode_torque(1000A): 钳位到 int16 上限 0x7FFF");
}

void test_encode_position() {
    // 手册示例：360° → 36000，限速 500dps → 0x01F4
    const auto f = motor_can::encode_position(1, 360.0, 500);
    const uint8_t want[8] = {0xA4, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0x00, 0x00};
    expect(same_data(f.data, want), "encode_position(360°, 500dps): 与手册示例逐字节一致");

    // -360° → 0xFFFF7360
    const auto g = motor_can::encode_position(1, -360.0, 500);
    const uint8_t want_neg[8] = {0xA4, 0x00, 0xF4, 0x01, 0x60, 0x73, 0xFF, 0xFF};
    expect(same_data(g.data, want_neg), "encode_position(-360°): 负数 int32 小端打包");
}

// ---- 解帧（收方向）----

void test_decode_status() {
    // 手册示例回复：temp=50℃, MOS=0℃, 抱闸已释放, 电压 0x01E5=48.5V, 错误 0x0004(低压)
    const uint8_t payload[7] = {0x32, 0x00, 0x01, 0xE5, 0x01, 0x04, 0x00};
    motor_can::MotorStatus st;
    expect(motor_can::decode_status(reply_frame(0x9A, payload), st), "decode_status: 正常回复");
    expect(st.temp_c == 50 && st.mos_temp_c == 0, "decode_status: 温度字段");
    expect(st.brake_released, "decode_status: 抱闸已释放");
    expect(near(st.voltage_v, 48.5), "decode_status: 电压 0x01E5=48.5V");
    expect(st.error_state == 0x0004, "decode_status: 错误标志 0x0004");

    // 异常：命令字节不符 / DLC 不足
    expect(!motor_can::decode_status(reply_frame(0x9C, payload), st), "decode_status: 命令字节不符拒绝");
    CanFrame bad = reply_frame(0x9A, payload);
    bad.dlc = 4;
    expect(!motor_can::decode_status(bad, st), "decode_status: DLC<8 拒绝");
}

void test_decode_run_status() {
    // 手册 0x9C 示例回复：temp=50℃, iq=0x0064=1A, speed=0x01F4=500dps, angle=0x002D=45°
    const uint8_t payload[7] = {0x32, 0x64, 0x00, 0xF4, 0x01, 0x2D, 0x00};
    motor_can::MotorRunStatus st;
    expect(motor_can::decode_run_status(reply_frame(0x9C, payload), st), "decode_run_status: 0x9C 回复");
    expect(near(st.iq_a, 1.0) && near(st.speed_dps, 500.0) && near(st.angle_deg, 45.0),
           "decode_run_status: 电流/转速/角度数值");

    // 控制命令回复布局一致：0xA2/0xA8/0xA9 也接受
    expect(motor_can::decode_run_status(reply_frame(0xA2, payload), st),
           "decode_run_status: 控制命令 0xA2 回复也接受");
    expect(motor_can::decode_run_status(reply_frame(0xA8, payload), st),
           "decode_run_status: 增量位置 0xA8 回复也接受");
    expect(motor_can::decode_run_status(reply_frame(0xA9, payload), st),
           "decode_run_status: 力控位置 0xA9 回复也接受");

    // 异常：非运行状态命令拒绝
    const uint8_t bad_payload[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    expect(!motor_can::decode_run_status(reply_frame(0x77, bad_payload), st),
           "decode_run_status: 0x77 回复拒绝");
}

void test_decode_angle() {
    // 手册示例：0x00008CA0 = 36000 → 360°
    const uint8_t payload[7] = {0x00, 0x00, 0x00, 0xA0, 0x8C, 0x00, 0x00};
    double angle = 0.0;
    expect(motor_can::decode_angle(reply_frame(0x92, payload), angle), "decode_angle: 正常回复");
    expect(near(angle, 360.0), "decode_angle: 0x00008CA0=360°");

    expect(!motor_can::decode_angle(reply_frame(0x93, payload), angle), "decode_angle: 命令字节不符拒绝");
}

// ---- 第二批：PID / 加速度 / 编码器 / 增量·力控位置 / 单圈角度 / 模式 ----

void test_encode_pid() {
    // 读 PID：索引放 Data[1]（SpeedKp=0x04）
    const auto r = motor_can::encode_read_pid(1, motor_can::PidIndex::SpeedKp);
    const uint8_t want_read[8] = {0x30, 0x04, 0, 0, 0, 0, 0, 0};
    expect(same_data(r.data, want_read), "encode_read_pid(SpeedKp): 索引在 Data[1]");

    // 写 PID 到 RAM：1.5f → 0x3FC00000 小端（手册示例）
    const auto w = motor_can::encode_write_pid_ram(1, motor_can::PidIndex::CurrentKp, 1.5);
    const uint8_t want_ram[8] = {0x31, 0x01, 0, 0, 0x00, 0x00, 0xC0, 0x3F};
    expect(same_data(w.data, want_ram), "encode_write_pid_ram(CurrentKp, 1.5): 与手册示例逐字节一致");

    // 写 PID 到 ROM：同一数值，命令字节 0x32
    const auto rom = motor_can::encode_write_pid_rom(1, motor_can::PidIndex::CurrentKp, 1.5);
    const uint8_t want_rom[8] = {0x32, 0x01, 0, 0, 0x00, 0x00, 0xC0, 0x3F};
    expect(same_data(rom.data, want_rom), "encode_write_pid_rom(CurrentKp, 1.5): 命令字节 0x32");
}

void test_decode_pid() {
    // 回复 float32：0x3F800000 = 1.0f
    const uint8_t payload[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F};
    float value = 0.0f;
    expect(motor_can::decode_pid(reply_frame(0x30, payload), value), "decode_pid: 正常回复");
    expect(near(value, 1.0), "decode_pid: 0x3F800000 = 1.0");

    // 写命令回显也能解（0x31/0x32 同布局）
    const auto echo = motor_can::encode_write_pid_rom(1, motor_can::PidIndex::CurrentKp, 1.5);
    value = 0.0f;
    expect(motor_can::decode_pid(echo, value) && near(value, 1.5),
           "decode_pid: 写命令回显往返，1.5 位域还原");

    expect(!motor_can::decode_pid(reply_frame(0x33, payload), value), "decode_pid: 命令字节不符拒绝");
}

void test_encode_accel() {
    // 读加速度：索引放 Data[1]（PositionAccel=0x00）
    const auto r = motor_can::encode_read_accel(1, motor_can::AccelIndex::PositionAccel);
    const uint8_t want_read[8] = {0x42, 0x00, 0, 0, 0, 0, 0, 0};
    expect(same_data(r.data, want_read), "encode_read_accel(PositionAccel): 索引在 Data[1]");

    // 手册示例：写速度规划加速度 10000 → 0x00002710
    const auto w = motor_can::encode_write_accel(1, motor_can::AccelIndex::SpeedAccel, 10000.0);
    const uint8_t want[8] = {0x43, 0x02, 0, 0, 0x10, 0x27, 0x00, 0x00};
    expect(same_data(w.data, want), "encode_write_accel(SpeedAccel, 10000): 与手册示例逐字节一致");

    // 钳位：50 → 下限 100；70000 → 上限 60000
    const auto lo = motor_can::encode_write_accel(1, motor_can::AccelIndex::SpeedAccel, 50.0);
    const uint8_t want_lo[8] = {0x43, 0x02, 0, 0, 0x64, 0x00, 0x00, 0x00};
    expect(same_data(lo.data, want_lo), "encode_write_accel(50): 钳位到下限 100");
    const auto hi = motor_can::encode_write_accel(1, motor_can::AccelIndex::SpeedAccel, 70000.0);
    const uint8_t want_hi[8] = {0x43, 0x02, 0, 0, 0x60, 0xEA, 0x00, 0x00};
    expect(same_data(hi.data, want_hi), "encode_write_accel(70000): 钳位到上限 60000");
}

void test_decode_accel() {
    // 回复 0x00002710 = 10000 → 10000°/s²
    const uint8_t payload[7] = {0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
    double value = 0.0;
    expect(motor_can::decode_accel(reply_frame(0x42, payload), value), "decode_accel: 正常回复");
    expect(near(value, 10000.0), "decode_accel: 0x00002710 = 10000°/s²");

    expect(!motor_can::decode_accel(reply_frame(0x44, payload), value), "decode_accel: 命令字节不符拒绝");
}

void test_encode_encoder() {
    // 只读查询：0x60 编码器位置
    const auto q = motor_can::encode_command(1, motor_can::RhCmd::EncoderPos);
    const uint8_t want_query[8] = {0x60, 0, 0, 0, 0, 0, 0, 0};
    expect(same_data(q.data, want_query), "encode_command(0x60): 编码器位置查询");

    // 写多圈零偏 0x63：Data[4..7] = 10000
    const auto o = motor_can::encode_write_encoder_offset(1, 10000);
    const uint8_t want[8] = {0x63, 0, 0, 0, 0x10, 0x27, 0x00, 0x00};
    expect(same_data(o.data, want), "encode_write_encoder_offset(10000): int32 小端打包");
}

void test_decode_encoder() {
    // 0x60 多圈位置：0x00002710 = 10000 脉冲
    const uint8_t payload[7] = {0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
    int32_t pos = 0;
    expect(motor_can::decode_encoder_position(reply_frame(0x60, payload), pos), "decode_encoder_position: 0x60");
    expect(pos == 10000, "decode_encoder_position: 10000 脉冲");

    // 0x62 零偏负数：-10000 → 0xFFFFD8F0
    const uint8_t neg_payload[7] = {0x00, 0x00, 0x00, 0xF0, 0xD8, 0xFF, 0xFF};
    expect(motor_can::decode_encoder_position(reply_frame(0x62, neg_payload), pos) && pos == -10000,
           "decode_encoder_position: 0x62 负零偏 -10000");

    // 写命令回显 0x63/0x64 布局相同，也接受（与 decode_pid 收 0x31/0x32 一致）
    expect(motor_can::decode_encoder_position(reply_frame(0x63, payload), pos) && pos == 10000,
           "decode_encoder_position: 0x63 写零偏回显也接受");
    expect(motor_can::decode_encoder_position(reply_frame(0x64, payload), pos) && pos == 10000,
           "decode_encoder_position: 0x64 零点写回显也接受");

    // 其余命令字节拒绝
    expect(!motor_can::decode_encoder_position(reply_frame(0x65, payload), pos),
           "decode_encoder_position: 0x65 拒绝");
}

void test_encode_position_batch2() {
    // 手册示例：增量位置 0xA8，+360° 限速 500dps
    const auto inc = motor_can::encode_increment_position(1, 360.0, 500);
    const uint8_t want_inc[8] = {0xA8, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0x00, 0x00};
    expect(same_data(inc.data, want_inc), "encode_increment_position(360°, 500dps): 与手册示例逐字节一致");

    // 手册示例：力控位置 0xA9，+360° 限速 500dps，最大扭矩 60%
    const auto force = motor_can::encode_force_position(1, 360.0, 500, 60);
    const uint8_t want_force[8] = {0xA9, 0x3C, 0xF4, 0x01, 0xA0, 0x8C, 0x00, 0x00};
    expect(same_data(force.data, want_force), "encode_force_position(360°, 500dps, 60%): 与手册示例逐字节一致");
}

void test_decode_single_angle() {
    // 手册示例：0x2710 = 10000 → 100.00°
    const uint8_t payload[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27};
    double angle = 0.0;
    expect(motor_can::decode_single_angle(reply_frame(0x94, payload), angle), "decode_single_angle: 正常回复");
    expect(near(angle, 100.0), "decode_single_angle: 0x2710 = 100°");

    expect(!motor_can::decode_single_angle(reply_frame(0x93, payload), angle),
           "decode_single_angle: 命令字节不符拒绝");
}

void test_decode_mode() {
    // Data[7] = 0x03 → 位置环
    const uint8_t payload[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
    motor_can::RunMode mode;
    expect(motor_can::decode_mode(reply_frame(0x70, payload), mode), "decode_mode: 正常回复");
    expect(mode == motor_can::RunMode::Position, "decode_mode: 0x03 = 位置环模式");

    // 非法模式值拒绝
    const uint8_t bad_payload[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
    expect(!motor_can::decode_mode(reply_frame(0x70, bad_payload), mode), "decode_mode: 非法模式值 0x04 拒绝");
    expect(!motor_can::decode_mode(reply_frame(0x71, payload), mode), "decode_mode: 命令字节不符拒绝");
}

void test_encode_set_can_id() {
    // 实测帧：ID 1 的电机改成 ID 2 → 141#2005000002000000（0x20 索引 0x05，Data[4..7]=2）
    const auto f = motor_can::encode_set_can_id(1, 2);
    expect(f.id == 0x141 && !f.is_extended && f.dlc == 8, "encode_set_can_id: 按地址 0x140+current_id/标准帧/DLC=8");
    const uint8_t want[8] = {0x20, 0x05, 0, 0, 2, 0, 0, 0};
    expect(same_data(f.data, want), "encode_set_can_id(1,2): 0x20 索引 0x05，Data[4..7]=2 (uint32 LE)");

    // 最大 ID 32：Data[4]=0x20，高位清零
    const auto f32 = motor_can::encode_set_can_id(7, 32);
    const uint8_t want32[8] = {0x20, 0x05, 0, 0, 0x20, 0, 0, 0};
    expect(same_data(f32.data, want32), "encode_set_can_id(7,32): Data[4]=0x20，其余清零");
}

void test_decode_set_can_id() {
    // 回显：命令 0x20、索引 0x05、Data[4..7]=2
    const uint8_t payload[7] = {0x05, 0, 0, 2, 0, 0, 0};
    uint8_t id = 0;
    expect(motor_can::decode_set_can_id(reply_frame(0x20, payload), id), "decode_set_can_id: 正常回显");
    expect(id == 2, "decode_set_can_id: Data[4..7]=2 → ID 2");

    // 命令字节不符拒绝
    expect(!motor_can::decode_set_can_id(reply_frame(0x21, payload), id), "decode_set_can_id: 命令字节不符拒绝");

    // 功能索引不符（0x06）拒绝
    const uint8_t bad_index_payload[7] = {0x06, 0, 0, 2, 0, 0, 0};
    expect(!motor_can::decode_set_can_id(reply_frame(0x20, bad_index_payload), id), "decode_set_can_id: 功能索引不符拒绝");

    // 超范围拒绝（值 0）
    const uint8_t zero_payload[7] = {0x05, 0, 0, 0, 0, 0, 0};
    expect(!motor_can::decode_set_can_id(reply_frame(0x20, zero_payload), id), "decode_set_can_id: ID 超范围（0）拒绝");

    // 超范围拒绝（值 33）
    const uint8_t high_payload[7] = {0x05, 0, 0, 33, 0, 0, 0};
    expect(!motor_can::decode_set_can_id(reply_frame(0x20, high_payload), id), "decode_set_can_id: ID 超范围（33）拒绝");
}

void test_encode_clear_multi_turn() {
    // 手册示例：0x20 索引 0x01，Data[2..7]=0 → 141#2001000000000000
    const auto f = motor_can::encode_clear_multi_turn(1);
    expect(f.id == 0x141 && !f.is_extended && f.dlc == 8,
           "encode_clear_multi_turn: 0x140+ID/标准帧/DLC=8");
    const uint8_t want[8] = {0x20, 0x01, 0, 0, 0, 0, 0, 0};
    expect(same_data(f.data, want), "encode_clear_multi_turn(1): 与手册示例逐字节一致");
}

void test_encode_active_report() {
    // 手册示例：使能 0x60 主动回复，间隔 10ms → 141#B660010100000000
    const auto f = motor_can::encode_active_report(1, 0x60, true, 1);
    const uint8_t want[8] = {0xB6, 0x60, 0x01, 0x01, 0, 0, 0, 0};
    expect(same_data(f.data, want), "encode_active_report(0x60, 使能, 10ms): 与手册示例逐字节一致");

    // 0x9C 使能，间隔 30×10ms=300ms → Data[3]=0x1E
    const auto g = motor_can::encode_active_report(1, 0x9C, true, 30);
    const uint8_t want_300[8] = {0xB6, 0x9C, 0x01, 0x1E, 0, 0, 0, 0};
    expect(same_data(g.data, want_300), "encode_active_report(0x9C, 使能, 300ms): 间隔 30 放 Data[3]");

    // 关闭 0x9A：Data[2]=0
    const auto off = motor_can::encode_active_report(1, 0x9A, false, 30);
    const uint8_t want_off[8] = {0xB6, 0x9A, 0x00, 0x1E, 0, 0, 0, 0};
    expect(same_data(off.data, want_off), "encode_active_report(0x9A, 关闭): Data[2]=0");
}

void test_decode_clear_multi_turn() {
    // 合法回显：命令 0x20、索引 0x01
    const uint8_t ok_payload[7] = {0x01, 0, 0, 0, 0, 0, 0};
    expect(motor_can::decode_clear_multi_turn(reply_frame(0x20, ok_payload)),
           "decode_clear_multi_turn: 正常回显");

    // 命令字节不符拒绝
    expect(!motor_can::decode_clear_multi_turn(reply_frame(0x21, ok_payload)),
           "decode_clear_multi_turn: 命令字节不符拒绝");

    // 功能索引不符（0x05）拒绝
    const uint8_t wrong_index_payload[7] = {0x05, 0, 0, 0, 0, 0, 0};
    expect(!motor_can::decode_clear_multi_turn(reply_frame(0x20, wrong_index_payload)),
           "decode_clear_multi_turn: 功能索引不符拒绝");

    // DLC 不足拒绝
    CanFrame bad = reply_frame(0x20, ok_payload);
    bad.dlc = 4;
    expect(!motor_can::decode_clear_multi_turn(bad), "decode_clear_multi_turn: DLC<8 拒绝");
}

// ---- 第三批：0x20 全索引 / 0x90 / 0x9D / 0xA6 / 0xB1~0xB5 / 0x280 ----

void test_encode_function_control() {
    // 手册示例：CANID 滤波器使能 0x20 索引 0x02，Value=1 → 141#2002000001000000
    const auto f = motor_can::encode_function_control(1, motor_can::FunctionIndex::CanIdFilter, 1);
    const uint8_t want[8] = {0x20, 0x02, 0, 0, 1, 0, 0, 0};
    expect(same_data(f.data, want), "encode_function_control(CanIdFilter, 1): 与手册示例逐字节一致");

    // 其余索引：错误上报 0x03、掉电保存 0x04、最大正角度 0x06（Value=36000）
    const auto e = motor_can::encode_function_control(1, motor_can::FunctionIndex::ErrorReport, 1);
    const uint8_t want_e[8] = {0x20, 0x03, 0, 0, 1, 0, 0, 0};
    expect(same_data(e.data, want_e), "encode_function_control(ErrorReport, 1): 索引 0x03");
    const auto m = motor_can::encode_function_control(
        1, motor_can::FunctionIndex::MultiTurnPowerSave, 1);
    const uint8_t want_m[8] = {0x20, 0x04, 0, 0, 1, 0, 0, 0};
    expect(same_data(m.data, want_m), "encode_function_control(MultiTurnPowerSave, 1): 索引 0x04");
    const auto p = motor_can::encode_function_control(
        1, motor_can::FunctionIndex::MaxPosAngle, 36000);
    const uint8_t want_p[8] = {0x20, 0x06, 0, 0, 0xA0, 0x8C, 0, 0};
    expect(same_data(p.data, want_p), "encode_function_control(MaxPosAngle, 36000): Value 小端打包");
}

void test_encode_com_protect_time() {
    // 手册示例 2：1000ms → Data[4..7]=0x000003E8
    const auto f = motor_can::encode_com_protect_time(1, 1000);
    const uint8_t want[8] = {0xB3, 0, 0, 0, 0xE8, 0x03, 0, 0};
    expect(same_data(f.data, want), "encode_com_protect_time(1000ms): 与手册示例逐字节一致");

    // 0 = 不使能保护
    const auto off = motor_can::encode_com_protect_time(1, 0);
    const uint8_t want_off[8] = {0xB3, 0, 0, 0, 0, 0, 0, 0};
    expect(same_data(off.data, want_off), "encode_com_protect_time(0): 关闭保护");
}

void test_encode_baudrate() {
    // 手册示例 2：Data[7]=1（CAN 1Mbps）
    const auto f = motor_can::encode_set_baudrate(1, 1);
    const uint8_t want[8] = {0xB4, 0, 0, 0, 0, 0, 0, 0x01};
    expect(same_data(f.data, want), "encode_set_baudrate(1): Data[7]=1");
    const auto g = motor_can::encode_set_baudrate(1, 2);
    expect(g.data[7] == 0x02, "encode_set_baudrate(2): Data[7]=2");
}

void test_encode_single_angle_position() {
    // 手册示例 1：顺时针 0、限速 500dps、360° → 141#A600F401A08C0000
    const auto f = motor_can::encode_single_angle_position(1, 0, 500, 360.0);
    const uint8_t want[8] = {0xA6, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0, 0};
    expect(same_data(f.data, want), "encode_single_angle_position(顺, 500dps, 360°): 与手册示例逐字节一致");

    // 逆时针方向位 Data[1]=1
    const auto g = motor_can::encode_single_angle_position(1, 1, 500, 360.0);
    expect(g.data[1] == 0x01, "encode_single_angle_position(逆): Data[1]=1");

    // 超范围钳位：540° → uint16 上限 35999
    const auto h = motor_can::encode_single_angle_position(1, 0, 500, 540.0);
    const uint8_t want_clamp[8] = {0xA6, 0x00, 0xF4, 0x01, 0x9F, 0x8C, 0, 0};  // 35999=0x8C9F
    expect(same_data(h.data, want_clamp), "encode_single_angle_position(540°): 钳位到 35999");
}

void test_to_broadcast() {
    // 单机停止帧转广播：ID=0x280，数据不变
    const auto stop = motor_can::to_broadcast(motor_can::encode_command(1, motor_can::RhCmd::MotorStop));
    expect(stop.id == 0x280 && !stop.is_extended && stop.dlc == 8 && stop.data[0] == 0x81,
           "to_broadcast(0x81): ID=0x280/标准帧/命令字节保留");

    // 位置指令广播：数据域与单机一致（手册示例 360° 500dps）
    const auto pos = motor_can::to_broadcast(motor_can::encode_position(1, 360.0, 500));
    const uint8_t want[8] = {0xA4, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0, 0};
    expect(pos.id == 0x280 && same_data(pos.data, want), "to_broadcast(0xA4): 数据域与单机一致");
}

void test_decode_function_control_echo() {
    // 合法回显：命令 0x20、索引 0x02、Value=1
    const uint8_t ok_payload[7] = {0x02, 0, 0, 1, 0, 0, 0};
    uint32_t value = 0;
    expect(motor_can::decode_function_control_echo(reply_frame(0x20, ok_payload),
                                                   motor_can::FunctionIndex::CanIdFilter, value),
           "decode_function_control_echo: 正常回显");
    expect(value == 1, "decode_function_control_echo: Value 回读=1");

    // 索引不符拒绝
    const uint8_t wrong_index_payload[7] = {0x05, 0, 0, 2, 0, 0, 0};
    expect(!motor_can::decode_function_control_echo(reply_frame(0x20, wrong_index_payload),
                                                    motor_can::FunctionIndex::CanIdFilter, value),
           "decode_function_control_echo: 索引不符拒绝");
    // 命令字节不符拒绝
    expect(!motor_can::decode_function_control_echo(reply_frame(0x21, ok_payload),
                                                    motor_can::FunctionIndex::CanIdFilter, value),
           "decode_function_control_echo: 命令字节不符拒绝");
}

void test_decode_status3() {
    // 手册示例：50℃、iA=0x0BC2=30.1A、iB=0xFA10=-15.2A、iC=0xF9C0=-16A
    const uint8_t payload[7] = {0x32, 0xC2, 0x0B, 0x10, 0xFA, 0xC0, 0xF9};
    motor_can::MotorStatus3 st;
    expect(motor_can::decode_status3(reply_frame(0x9D, payload), st), "decode_status3: 正常回复");
    expect(st.temp_c == 50, "decode_status3: 温度 50℃");
    expect(near(st.ia_a, 30.1) && near(st.ib_a, -15.2) && near(st.ic_a, -16.0),
           "decode_status3: 三相相电流 30.1/-15.2/-16A");

    expect(!motor_can::decode_status3(reply_frame(0x9E, payload), st), "decode_status3: 命令字节不符拒绝");
}

void test_decode_single_encoder() {
    // 手册示例：encoder=0x0833=2099、raw=0x2CBE=11454、offset=0x248B=9355
    const uint8_t payload[7] = {0x00, 0x33, 0x08, 0xBE, 0x2C, 0x8B, 0x24};
    motor_can::SingleEncoder enc;
    expect(motor_can::decode_single_encoder(reply_frame(0x90, payload), enc), "decode_single_encoder: 正常回复");
    expect(enc.encoder == 2099 && enc.raw == 11454 && enc.offset == 9355,
           "decode_single_encoder: encoder/raw/offset 与手册示例一致");

    expect(!motor_can::decode_single_encoder(reply_frame(0x91, payload), enc),
           "decode_single_encoder: 命令字节不符拒绝");
}

void test_decode_run_time() {
    // 手册示例：Data[4..7]=0x10000000 = 268435456ms（约 74 小时）
    const uint8_t payload[7] = {0, 0, 0, 0, 0, 0, 0x10};
    uint32_t t = 0;
    expect(motor_can::decode_run_time(reply_frame(0xB1, payload), t), "decode_run_time: 正常回复");
    expect(t == 268435456, "decode_run_time: 0x10000000 = 268435456ms");

    expect(!motor_can::decode_run_time(reply_frame(0xB0, payload), t), "decode_run_time: 命令字节不符拒绝");
}

void test_decode_version_date() {
    // 手册示例：0x0134892E = 20220206
    const uint8_t payload[7] = {0, 0, 0, 0x2E, 0x89, 0x34, 0x01};
    uint32_t date = 0;
    expect(motor_can::decode_version_date(reply_frame(0xB2, payload), date), "decode_version_date: 正常回复");
    expect(date == 20220206, "decode_version_date: 0x0134892E = 20220206");

    expect(!motor_can::decode_version_date(reply_frame(0xB3, payload), date),
           "decode_version_date: 命令字节不符拒绝");
}

void test_decode_motor_model() {
    // 手册示例：58 38 53 32 56 31 30 = "X8S2V10"
    const uint8_t payload[7] = {0x58, 0x38, 0x53, 0x32, 0x56, 0x31, 0x30};
    char model[8] = {};
    expect(motor_can::decode_motor_model(reply_frame(0xB5, payload), model), "decode_motor_model: 正常回复");
    expect(model[0] == 'X' && model[1] == '8' && model[2] == 'S' && model[3] == '2' &&
               model[4] == 'V' && model[5] == '1' && model[6] == '0' && model[7] == '\0',
           "decode_motor_model: X8S2V10 + 末尾 '\\0'");

    expect(!motor_can::decode_motor_model(reply_frame(0xB6, payload), model),
           "decode_motor_model: 命令字节不符拒绝");
}

void test_decode_run_status_a6() {
    // 0xA6 回复布局同 0x9C，也接受（末字段为单圈编码器值）
    const uint8_t payload[7] = {0x32, 0x64, 0x00, 0xF4, 0x01, 0xE8, 0x03};
    motor_can::MotorRunStatus st;
    expect(motor_can::decode_run_status(reply_frame(0xA6, payload), st), "decode_run_status: 0xA6 回复也接受");
}

}  // namespace

int main() {
    test_encode_command();
    test_encode_speed();
    test_encode_torque();
    test_encode_position();
    test_decode_status();
    test_decode_run_status();
    test_decode_angle();
    test_encode_pid();
    test_decode_pid();
    test_encode_accel();
    test_decode_accel();
    test_encode_encoder();
    test_decode_encoder();
    test_encode_position_batch2();
    test_decode_single_angle();
    test_decode_mode();
    test_encode_set_can_id();
    test_decode_set_can_id();
    test_encode_clear_multi_turn();
    test_encode_active_report();
    test_decode_clear_multi_turn();
    test_encode_function_control();
    test_encode_com_protect_time();
    test_encode_baudrate();
    test_encode_single_angle_position();
    test_to_broadcast();
    test_decode_function_control_echo();
    test_decode_status3();
    test_decode_single_encoder();
    test_decode_run_time();
    test_decode_version_date();
    test_decode_motor_model();
    test_decode_run_status_a6();

    if (g_failures > 0) {
        MC_LOG_ERROR("%d 个用例失败", g_failures);
        return 1;
    }
    MC_LOG_INFO("全部用例通过");
    return 0;
}
