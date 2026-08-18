// examples/single_motor/main.cpp
// 单机使用示例：Motor（Device 层）抽象接口的最小参考。
// 打开总线 → 建 Motor（不自动归0）→ 读状态 → 开闸 → 位置到 90° → 回读角度 → 停止 → 锁闸。
//
// 构建:
//   cd build && cmake .. && make -j example_single_motor
// 用法:
//   ./build/example_single_motor [ifname]        # 默认 can0
// 前置:
//   sudo ip link set can0 type can bitrate 1000000 && up；电机 24V 供电；CAN ID=1。
// 安全:
//   会真实驱动电机到 90°，运行前确认关节活动范围内无人、无障碍。

#include "motor_can/motor/motor.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char** argv) {
    const char* ifname = (argc > 1) ? argv[1] : "can0";

    motor_can::CanConfig cfg;
    cfg.ifname = ifname;
    motor_can::CanComm comm;
    if (!comm.open(cfg)) {
        std::fprintf(stderr,
                     "打开 %s 失败：先 `sudo ip link set %s type can bitrate 1000000 && up`\n",
                     ifname, ifname);
        return 1;
    }

    // 不自动归0（home_on_init=false），手动控制每一步时序
    motor_can::Motor::Config mcfg;
    mcfg.home_on_init = false;
    mcfg.max_speed_dps = 60;  // 位置环限速 60°/s
    motor_can::Motor motor(comm, 1, mcfg);

    // 读状态：确认电机在线
    motor_can::MotorStatus st;
    if (!motor.read_status(st)) {
        std::fprintf(stderr, "电机无响应：请检查 24V 供电与 CAN 接线\n");
        return 1;
    }
    std::printf("在线：温度 %d ℃，电压 %.1f V，抱闸 %s\n", st.temp_c, st.voltage_v,
                st.brake_released ? "释放" : "锁死");

    // 带抱闸电机运动前必须先开闸
    if (!motor.brake_release()) {
        std::fprintf(stderr, "开闸失败\n");
        return 1;
    }

    // 位置环：运动到 90°，限速用上面配置的 60°/s
    if (!motor.set_position(90.0)) {
        std::fprintf(stderr, "位置指令失败\n");
        return 1;
    }
    std::printf("已发送位置指令到 90°，等待运动完成…\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 回读当前角度
    double angle = 0.0;
    if (motor.read_angle(angle)) {
        std::printf("当前角度 %.2f°\n", angle);
    }

    // 停止并锁闸
    motor.stop();
    motor.brake_lock();
    return 0;
}
