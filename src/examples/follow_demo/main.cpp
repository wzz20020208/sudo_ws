// examples/follow_demo/main.cpp
// 跟随示例：电机 2 跟随电机 1 的运动（主从跟随）。
// 电机1 由外部单独控制（手推 / 另一程序 / 上位机），本程序绝不驱动电机1，
// 以 100Hz 读电机1/电机2 实时角度，电机2 去追（不人为限速），同一行刷新跟随误差。
// 实测：100Hz 可同步；更新率降到 10Hz 后误差极大（电机2 追移动目标落后 100ms 的指令）。
// 跟随持续到按回车键结束，结束时停止电机2 并锁闸。
// 全程走 Device 层 submit_*/read_angle 抽象接口，不使用 0x280 广播。
//
// 构建:
//   cd build && cmake .. && make -j example_follow_demo
// 用法:
//   ./build/example_follow_demo [ifname]        # 默认 can0
// 前置:
//   总线 up + 供电；默认 CAN ID 1 为主（外部控制）、2 为从，按实际接法改
//   kMotor1Id/kMotor2Id。
// 安全:
//   电机2 会真实运动，运行前确认活动范围内无人；按回车结束并自动锁闸。
//   跟随期间无时长上限，请勿无人看管；若用 Ctrl-C 中断，电机2 不会自动锁闸。

#include "motor_can/motor/multi_motor.hpp"

#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <poll.h>
#include <thread>
#include <unistd.h>

namespace {

constexpr uint8_t kMotor1Id = 1;  // 主：外部控制，本程序只读
constexpr uint8_t kMotor2Id = 2;  // 从：本程序驱动其跟随

// 位置环 max_speed 字段是必填，置大值等效不限速（X2-7 关节模组实际最高速远低于此，
// 真正限速由电机自身决定），让电机2 尽可能贴近电机1
constexpr uint16_t kNoSpeedLimitDps = 1000;

constexpr auto kPeriod = std::chrono::milliseconds(10);  // 控制周期：100Hz
constexpr int kPrintEvery = 10;  // 每 10 次（100ms）刷新一次同一行显示

}  // namespace

int main(int argc, char** argv) {
    const char* ifname = (argc > 1) ? argv[1] : "can0";

    motor_can::CanConfig cfg;
    cfg.ifname = ifname;

    // 打开共享总线 + 2 个工作线程（电机1 与电机2 的收发各占一线程）
    motor_can::MultiMotorController ctrl(cfg, 2);
    if (!ctrl.is_open()) {
        std::fprintf(stderr,
                     "打开 %s 失败：先 `sudo ip link set %s type can bitrate 1000000 && up`\n",
                     ifname, ifname);
        return 1;
    }

    // 两台都不自动归0：电机1 完全交给外部控制；电机2 从当前位置开始跟随
    motor_can::Motor::Config mcfg;
    mcfg.home_on_init = false;
    // 丢帧时不等 500ms：正常回复亚毫秒级，10ms（=一个控制周期）内不到即视为丢帧，
    // 下一周期发新指令顶上——跟随不因丢帧卡死（丢帧后的等待没有意义）
    mcfg.reply_timeout = std::chrono::milliseconds(10);
    if (!ctrl.add_motor(kMotor1Id, mcfg) || !ctrl.add_motor(kMotor2Id, mcfg)) {
        std::fprintf(stderr, "添加电机失败（ID 已存在？）\n");
        return 1;
    }
    ctrl.submit_brake(kMotor2Id, true);  // 开闸：让电机2 能跟随运动
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // stdin 改非阻塞：循环里用 poll 检测回车键，不阻塞 100Hz 跟随
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    std::printf("开始跟随，电机1 由外部控制，电机2 不限速跟随；按回车键停止…\n");
    bool stop_requested = false;
    int cycle = 0;
    while (!stop_requested) {
        const auto t0 = std::chrono::steady_clock::now();

        double a1 = 0.0, a2 = 0.0;
        // 丢帧时读角度失败：不当作错误处理（不报错、不中断）——电机2 会继续执行
        // 上一周期目标，跟随循环保持 100Hz 不间断，下一周期读到新角度再顶上
        if (ctrl.read_angle(kMotor1Id, a1) && ctrl.read_angle(kMotor2Id, a2)) {
            ctrl.submit_position(kMotor2Id, a1, kNoSpeedLimitDps);  // 电机2 追电机1（不限速）
            if (cycle % kPrintEvery == 0) {
                // \r 覆盖本行刷新，不刷屏；误差 = 主 - 从
                std::printf("\r主 %.2f°  从 %.2f°  误差 %+.2f°", a1, a2, a1 - a2);
                std::fflush(stdout);
            }
        }
        ++cycle;

        // 检测回车：有输入就读走并请求停止
        struct pollfd pfd { STDIN_FILENO, POLLIN, 0 };
        if (poll(&pfd, 1, 0) > 0) {
            char ch;
            while (read(STDIN_FILENO, &ch, 1) == 1) {
            }
            stop_requested = true;
        }

        std::this_thread::sleep_until(t0 + kPeriod);  // 固定周期，贴近 100Hz
    }
    std::printf("\n");

    // 结束：停止电机2 并锁闸
    ctrl.submit_stop(kMotor2Id);
    ctrl.submit_brake(kMotor2Id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 等工作线程执行完
    std::printf("跟随结束，电机2 已停止并锁闸。\n");
    return 0;
}
