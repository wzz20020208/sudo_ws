// tests/motor_device_test.cpp
// 电机控制层实机测试（Device 层，需 CAN 接口 + 电机）：
//  Phase A - Motor 单机：开闸 → 正转 5s → 停 → 位置到 45° → 回 0 → 停止锁闸，
//            全程验证请求/回复过滤 + 三环接口（set_speed / set_position）。
//  Phase B - MultiMotorController 冒烟：add_motor 归0，线程池提交速度指令 → 读角度确认运动
//            → 回 0 → 停止锁闸。
// 两阶段各用独立 CanComm（单 socket），前一阶段销毁后才开下一阶段，避免同总线双 socket 串扰。
//
// 用法: ./motor_device_test [--ifname <can接口名>] [--id <电机ID>]
// 前置: sudo ip link set <ifname> type can bitrate 1000000 && up，并给电机供电。
// 安全: 会真实驱动电机，运动前确认关节范围内无人；带抱闸电机测试内自动开闸/锁闸。

#include "motor_can/common/log.hpp"
#include "motor_can/motor/motor.hpp"
#include "motor_can/motor/multi_motor.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// 轮询读角度直到到达目标（±1°）或超时
bool wait_angle(motor_can::Motor& motor, double target_deg, int timeout_ms) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        double angle = 0.0;
        if (motor.read_angle(angle) && std::fabs(angle - target_deg) < 1.0) {
            MC_LOG_INFO("已到达目标 %.0f°（当前 %.1f°）", target_deg, angle);
            return true;
        }
        sleep_ms(200);
    }
    MC_LOG_ERROR("等待到达 %.0f° 超时（当前位置未确认）", target_deg);
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    std::string ifname = "can0";
    int motor_id = 1;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--ifname") == 0 && i + 1 < argc) {
            ifname = argv[++i];
        } else if (std::strcmp(argv[i], "--id") == 0 && i + 1 < argc) {
            motor_id = std::atoi(argv[++i]);
        } else {
            MC_LOG_INFO("用法: %s [--ifname <can接口名>] [--id <电机ID>]", argv[0]);
            return 1;
        }
    }
    if (motor_id < 1 || motor_id > 32) {
        MC_LOG_ERROR("电机 ID 必须在 1..32 之间，当前 %d", motor_id);
        return 1;
    }

    bool ok = true;

    // ---- Phase A: Motor 单机 ----
    {
        motor_can::CanConfig cfg;
        cfg.ifname = ifname;
        motor_can::CanComm comm;
        if (!comm.open(cfg)) {
            MC_LOG_ERROR("open(%s) 失败", ifname.c_str());
            return 1;
        }

        motor_can::Motor::Config mcfg;
        mcfg.home_on_init = false;  // 手动控制时序，不自动归0
        mcfg.max_speed_dps = 15;    // 位置环限速 15dps
        mcfg.max_torque_pct = 30;   // 速度环限扭 30%
        motor_can::Motor motor(comm, motor_id, mcfg);

        motor_can::MotorStatus st;
        if (!motor.read_status(st)) {
            MC_LOG_ERROR("初始状态读取失败");
            return 1;
        }
        MC_LOG_INFO("初始状态: 温度%d℃ 电压%.1fV 抱闸%s 错误0x%04X", st.temp_c, st.voltage_v,
                    st.brake_released ? "释放" : "锁死", st.error_state);

        // 开闸（带抱闸电机运动前必须）
        if (!motor.brake_release()) {
            MC_LOG_ERROR("开闸失败");
            return 1;
        }
        sleep_ms(500);

        // 正转 5s
        motor_can::MotorRunStatus rs;
        if (motor.set_speed(10.0, &rs)) {
            MC_LOG_INFO("正转指令已发（转速 %.0fdps 电流 %.2fA）", rs.speed_dps, rs.iq_a);
        } else {
            MC_LOG_ERROR("正转指令失败");
            ok = false;
        }
        sleep_ms(5000);

        // 停 1s
        motor.set_speed(0.0, &rs);
        sleep_ms(1000);

        // 位置到 45°（限速 15dps）
        if (motor.set_position(45.0, &rs)) {
            MC_LOG_INFO("位置指令 45° 已发（转速 %.0fdps）", rs.speed_dps);
            ok &= wait_angle(motor, 45.0, 15000);
        } else {
            MC_LOG_ERROR("位置指令 45° 失败");
            ok = false;
        }

        // 回 0
        if (motor.set_position(0.0, &rs)) {
            ok &= wait_angle(motor, 0.0, 15000);
        } else {
            MC_LOG_ERROR("回 0 位置指令失败");
            ok = false;
        }

        // 停止锁闸（前面出错也执行，保证电机安全停下）
        motor.stop();
        sleep_ms(200);
        motor.brake_lock();
        sleep_ms(200);
        if (motor.read_status(st)) {
            MC_LOG_INFO("结束状态: 温度%d℃ 电压%.1fV 抱闸%s 错误0x%04X", st.temp_c, st.voltage_v,
                        st.brake_released ? "释放" : "锁死", st.error_state);
        }
    }  // Motor 与 comm 先销毁，再进 Phase B

    // ---- Phase B: MultiMotorController 冒烟（独立 socket） ----
    {
        motor_can::CanConfig cfg;
        cfg.ifname = ifname;
        motor_can::MultiMotorController ctrl(cfg, 4);
        if (!ctrl.is_open()) {
            MC_LOG_ERROR("MultiMotorController open 失败");
            return 1;
        }
        if (!ctrl.add_motor(static_cast<uint8_t>(motor_id))) {
            MC_LOG_ERROR("add_motor(%d) 失败", motor_id);
            return 1;
        }
        sleep_ms(500);  // add_motor 已物理归0（home_on_init=true）

        motor_can::MotorStatus st;
        if (!ctrl.read_status(motor_id, st)) {
            MC_LOG_ERROR("多机读状态失败");
            return 1;
        }
        MC_LOG_INFO("多机状态: 温度%d℃ 电压%.1fV 抱闸%s", st.temp_c, st.voltage_v,
                    st.brake_released ? "释放" : "锁死");

        // 线程池提交速度指令，等待后读角度确认运动
        double a0 = 0.0, a1 = 0.0;
        ctrl.read_angle(motor_id, a0);
        if (!ctrl.submit_speed(motor_id, 10.0, 30)) {
            MC_LOG_ERROR("submit_speed 入队失败");
            return 1;
        }
        sleep_ms(3000);
        ctrl.read_angle(motor_id, a1);
        MC_LOG_INFO("速度指令后角度 %.1f° -> %.1f°", a0, a1);
        ok &= std::fabs(a1 - a0) > 1.0;  // 角度有变化 → 指令经线程池真实发布

        // 回 0 + 停止锁闸（同一电机 FIFO 串行执行）
        ctrl.submit_position(motor_id, 0.0, 20);
        sleep_ms(3000);
        ctrl.submit_stop(motor_id);
        ctrl.submit_brake(motor_id, false);  // 锁闸
        sleep_ms(500);
    }

    if (!ok) {
        MC_LOG_ERROR("测试结束，但有环节未通过");
        return 1;
    }
    MC_LOG_INFO("测试结束，所有环节通过");
    return 0;
}
