// tests/motor_spin_test.cpp
// 电机缓慢转动测试（使用协议层 API 编解码）：抱闸释放 → 正转 5s → 停 1s → 反转 5s → 停。
// 每条指令发出后用 receive_by_id() 等待回复，decode_run_status 解出实时转速。
// 在初始 / 抱闸释放后 / 锁闸后三点用 decode_status 读取电机状态。
//
// 安全提示：
//  - 带抱闸电机运动前必须先发 0x77 释放抱闸；结束时 0x81 停止 + 0x78 锁闸。
//  - 运行前需配置 CAN 接口：sudo ip link set <ifname> type can bitrate 1000000 && up，
//    并给电机供电；测试时远离关节运动范围。
//
// 用法: ./motor_spin_test [--ifname <can接口名>] [--id <电机ID 1..32>]
//                        [--speed <转速dps>] [--duration <时长秒>]
// 默认: can0, 电机ID 1, 10dps, 每段 5 秒。

#include "motor_can/can_comm/can_comm.hpp"
#include "motor_can/common/log.hpp"
#include "motor_can/protocol/rh_protocol.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

// 速度闭环限幅：最大扭矩 = 额定电流的 30%（0xA2 指令 DATA[1]，单位 1%）
constexpr uint8_t kMaxTorquePct = 30;
constexpr auto kReplyTimeout = std::chrono::milliseconds(500);

// 发送一条指令并等待电机回复（0x240+ID）。按命令类别处理回复：
//  - 控制命令（0xA1/0xA2/0xA4）：回复为运行状态布局，用 decode_run_status 解出实时转速
//  - 单字节命令（0x77/0x78/0x80/0x81）：回复为原帧回显，仅校验命令字节一致
bool cmd_and_reply(motor_can::CanComm& comm, uint8_t motor_id,
                   const motor_can::CanFrame& frame, motor_can::RhCmd cmd,
                   const char* desc) {
    if (!comm.send(frame)) {
        MC_LOG_ERROR("%s 发送失败", desc);
        return false;
    }
    motor_can::CanFrame reply;
    if (!comm.receive_by_id(0x240u + motor_id, reply, kReplyTimeout)) {
        MC_LOG_ERROR("%s 未收到电机回复(0x%03X)", desc, 0x240u + motor_id);
        return false;
    }
    if (cmd == motor_can::RhCmd::Speed || cmd == motor_can::RhCmd::Torque ||
        cmd == motor_can::RhCmd::Position) {
        motor_can::MotorRunStatus st;
        if (!motor_can::decode_run_status(reply, st)) {
            MC_LOG_ERROR("%s 回复命令字节不符，无法解码", desc);
            return false;
        }
        MC_LOG_INFO("%s -> 回复（转速 %.0fdps 电流 %.2fA）", desc, st.speed_dps, st.iq_a);
    } else {
        if (reply.data[0] != static_cast<uint8_t>(cmd)) {
            MC_LOG_ERROR("%s 回复命令字节不符（期望 0x%02X）", desc, static_cast<uint8_t>(cmd));
            return false;
        }
        MC_LOG_INFO("%s -> 收到回复", desc);
    }
    return true;
}

// 读取电机状态（0x9A），decode_status 解析温度/电压/抱闸/错误
bool read_status(motor_can::CanComm& comm, uint8_t motor_id, const char* desc) {
    if (!comm.send(motor_can::encode_command(motor_id, motor_can::RhCmd::ReadStatus))) {
        MC_LOG_ERROR("%s 发送失败", desc);
        return false;
    }
    motor_can::CanFrame reply;
    if (!comm.receive_by_id(0x240u + motor_id, reply, kReplyTimeout)) {
        MC_LOG_ERROR("%s 未收到电机回复", desc);
        return false;
    }
    motor_can::MotorStatus st;
    if (!motor_can::decode_status(reply, st)) {
        MC_LOG_ERROR("%s 回复命令字节不符，无法解码", desc);
        return false;
    }
    MC_LOG_INFO("%s: 电机温度 %d℃  MOS温度 %d℃  电压 %.1fV  抱闸%s  错误 0x%04X",
                desc, st.temp_c, st.mos_temp_c, st.voltage_v,
                st.brake_released ? "已释放" : "锁死", st.error_state);
    return true;
}

void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace

int main(int argc, char** argv) {
    std::string ifname = "can0";
    int motor_id = 1;
    int speed_dps = 10;
    int duration_ms = 5000;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--ifname") == 0 && i + 1 < argc) {
            ifname = argv[++i];
        } else if (std::strcmp(argv[i], "--id") == 0 && i + 1 < argc) {
            motor_id = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--speed") == 0 && i + 1 < argc) {
            speed_dps = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            duration_ms = std::atoi(argv[++i]) * 1000;
        } else {
            MC_LOG_INFO("用法: %s [--ifname <can接口名>] [--id <电机ID 1..32>] "
                        "[--speed <转速dps>] [--duration <时长秒>]", argv[0]);
            return 1;
        }
    }
    if (motor_id < 1 || motor_id > 32) {
        MC_LOG_ERROR("电机 ID 必须在 1..32 之间，当前 %d", motor_id);
        return 1;
    }

    motor_can::CanConfig cfg;
    cfg.ifname = ifname;
    motor_can::CanComm comm;
    if (!comm.open(cfg)) {
        MC_LOG_ERROR("open(%s) 失败，请先 `sudo ip link set %s type can bitrate 1000000 && up`",
                     ifname.c_str(), ifname.c_str());
        return 1;
    }

    bool ok = true;

    // 0. 初始状态读取：确认电机在总线上且无故障，再继续
    if (!read_status(comm, motor_id, "初始状态")) {
        comm.close();
        return 1;
    }

    // 1. 抱闸释放（带抱闸电机运动前必须）；失败则中止，不转动
    if (!cmd_and_reply(comm, motor_id,
                       motor_can::encode_command(motor_id, motor_can::RhCmd::BrakeRelease),
                       motor_can::RhCmd::BrakeRelease, "抱闸释放(0x77)")) {
        comm.close();
        return 1;
    }
    sleep_ms(500);
    ok &= read_status(comm, motor_id, "抱闸释放后");

    // 2. 正转
    ok &= cmd_and_reply(comm, motor_id,
                        motor_can::encode_speed(motor_id, speed_dps, kMaxTorquePct),
                        motor_can::RhCmd::Speed, "正转指令");
    sleep_ms(duration_ms);

    // 3. 停 1s
    ok &= cmd_and_reply(comm, motor_id,
                        motor_can::encode_speed(motor_id, 0.0, kMaxTorquePct),
                        motor_can::RhCmd::Speed, "速度归零");
    sleep_ms(1000);

    // 4. 反转
    ok &= cmd_and_reply(comm, motor_id,
                        motor_can::encode_speed(motor_id, -speed_dps, kMaxTorquePct),
                        motor_can::RhCmd::Speed, "反转指令");
    sleep_ms(duration_ms);

    // 5. 停止并锁闸（前面出错也执行，保证电机安全停下）
    ok &= cmd_and_reply(comm, motor_id,
                        motor_can::encode_speed(motor_id, 0.0, kMaxTorquePct),
                        motor_can::RhCmd::Speed, "速度归零");
    sleep_ms(500);
    ok &= cmd_and_reply(comm, motor_id,
                        motor_can::encode_command(motor_id, motor_can::RhCmd::MotorStop),
                        motor_can::RhCmd::MotorStop, "电机停止(0x81)");
    sleep_ms(200);
    ok &= cmd_and_reply(comm, motor_id,
                        motor_can::encode_command(motor_id, motor_can::RhCmd::BrakeLock),
                        motor_can::RhCmd::BrakeLock, "抱闸锁死(0x78)");
    sleep_ms(200);
    ok &= read_status(comm, motor_id, "锁闸后");

    comm.close();
    if (!ok) {
        MC_LOG_ERROR("测试结束，但有指令未收到电机回复，请检查通讯/电机状态");
        return 1;
    }
    MC_LOG_INFO("测试结束，电机已停止并锁闸，所有指令均收到回复");
    return 0;
}
