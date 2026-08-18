// examples/multi_motor/main.cpp
// 多机使用示例：MultiMotorController（Device 层）批量提交抽象接口的参考。
// 共享总线 + 线程池：逐台 add_motor（不自动归0）→ 批量发布位置指令 → 逐台回读角度 → 批量停止。
// 按设计理念，多机控制全部走 submit_* 逐台发布，不使用 0x280 广播。
//
// 构建:
//   cd build && cmake .. && make -j example_multi_motor
// 用法:
//   ./build/example_multi_motor [ifname]        # 默认 can0
// 前置:
//   总线 up + 供电；默认操作 CAN ID 1、2，按实际接法改 kIds。
// 安全:
//   会真实驱动两台电机，运行前确认活动范围内无人。

#include "motor_can/motor/multi_motor.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

namespace {

// 本示例操作的两台电机 CAN ID
constexpr uint8_t kIds[] = {1, 2};
constexpr size_t kMotorCount = 2;  // 线程池工作线程数 = 电机台数

}  // namespace

int main(int argc, char** argv) {
    const char* ifname = (argc > 1) ? argv[1] : "can0";

    motor_can::CanConfig cfg;
    cfg.ifname = ifname;

    // 打开共享总线 + 线程池（每台电机一个工作线程，异步发布）
    motor_can::MultiMotorController ctrl(cfg, kMotorCount);
    if (!ctrl.is_open()) {
        std::fprintf(stderr,
                     "打开 %s 失败：先 `sudo ip link set %s type can bitrate 1000000 && up`\n",
                     ifname, ifname);
        return 1;
    }

    // 逐台添加电机，不自动归0
    motor_can::Motor::Config mcfg;
    mcfg.home_on_init = false;
    mcfg.max_speed_dps = 60;  // 位置环限速 60°/s
    for (uint8_t id : kIds) {
        if (!ctrl.add_motor(id, mcfg)) {
            std::fprintf(stderr, "添加电机 %u 失败（ID 已存在？）\n", id);
            return 1;
        }
    }

    // 批量提交（入队即返回，线程池异步发布到各电机）：ID1 → 90°，ID2 → 180°
    ctrl.submit_position(1, 90.0, 60);
    ctrl.submit_position(2, 180.0, 60);
    std::printf("已提交位置指令：电机 1 → 90°，电机 2 → 180°，等待运动…\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 同步回读每台当前角度（监控/测试用，不走线程池）
    for (uint8_t id : kIds) {
        double angle = 0.0;
        if (ctrl.read_angle(id, angle)) {
            std::printf("电机 %u 当前角度 %.2f°\n", id, angle);
        }
    }

    // 批量停止（逐台发布，不用广播）
    ctrl.submit_stop(1);
    ctrl.submit_stop(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 等工作线程执行完
    return 0;
}
