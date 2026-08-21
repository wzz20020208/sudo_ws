// motor_follower_node.cpp
// 订阅 /motor_status（主电机角度）→ 位置环 0xA4 驱动从机跟随。
// 主角度来自话题（motor_status_publisher 节点 100Hz 发布），本程序只驱动从机，不回读主。
//
// 用法: ros2 run motor_follower motor_follower_node --ifname can1 --id 1 --max-speed 1000 [--ff-gain 1.0]
// 参数: --ifname    从机所在 CAN 口（默认 can0；主从分两条总线时从机填 can1 等）
//       --id        从机 CAN ID（默认 1；从机在另一条总线上时 ID 与主相同也可，不同总线不冲突）
//       --max-speed 位置环限速 °/s（默认 1000，置大值等效不限速，同 follow_demo）
//       --ff-gain   差分速度前馈系数（秒；默认 1.0，0=关闭）。
//                   从机位置环近似纯 P，匀速跟随稳态滞后 ≈ 转速/Kp；把滞后当提前量补进指令：
//                   目标 = 主角度 + k×主角速度。k≈1/Kp 时误差归 0，k 过大/过小只留微小残差，
//                   不改变从机内部闭环稳定性。实测数据滞后 182°@178°/s → k≈1.0；若你们现场
//                   误差是 30°@180°/s 则 k≈0.17。
// 退出: Ctrl-C（SIGINT/SIGTERM）停止从机并锁闸。
// 安全: 从机会真实运动，运行前确认活动范围内无人。

#include "motor_can/can_comm/can_comm.hpp"
#include "motor_can/motor/motor.hpp"

#include "motor_status_publisher/msg/motor_status.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

namespace {

// 丢帧时不等 500ms：正常回复亚毫秒级，10ms（≈一个订阅周期）内不到即视为丢帧，
// 下一帧话题消息再顶上——跟随不因丢帧卡死（同 example_follow_demo 的做法）
constexpr std::chrono::milliseconds kReplyTimeout{10};
constexpr std::chrono::milliseconds kPrintPeriod{1000};  // 1Hz 显示主/从/误差

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 简单参数解析：--ifname <接口> --id <从机ID> --max-speed <限速°/s> --ff-gain <前馈系数s>
    // （与工程其它工具一致）
    std::string ifname = "can0";
    uint8_t motor_id = 1;
    uint16_t max_speed = 500;
    double ff_gain_s = 1.0;  // 差分速度前馈系数（秒）；0=关闭
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--ifname") == 0 && i + 1 < argc) {
            ifname = argv[++i];
        } else if (std::strcmp(argv[i], "--id") == 0 && i + 1 < argc) {
            motor_id = static_cast<uint8_t>(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--max-speed") == 0 && i + 1 < argc) {
            max_speed = static_cast<uint16_t>(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--ff-gain") == 0 && i + 1 < argc) {
            ff_gain_s = std::atof(argv[++i]);
        }
    }

    motor_can::CanConfig cfg;
    cfg.ifname = ifname;
    motor_can::CanComm comm;
    if (!comm.open(cfg)) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_follower"),
                     "open(%s) 失败，请先配置 CAN 接口", ifname.c_str());
        return 1;
    }

    // 从机控制句柄：不归0（从当前位置开始跟随），reply_timeout=10ms 保 100Hz 节奏
    motor_can::Motor::Config mcfg;
    mcfg.home_on_init = false;
    mcfg.reply_timeout = kReplyTimeout;
    motor_can::Motor motor(comm, motor_id, mcfg);

    if (!motor.brake_release()) {
        RCLCPP_WARN(rclcpp::get_logger("motor_follower"),
                    "从机 %u 开闸失败（无回复），跟随可能无法运动，请确认供电/接线", motor_id);
    }

    auto node = std::make_shared<rclcpp::Node>("motor_follower");
    // QoS(10) 与发布端一致（keep_last 10, reliable）：100Hz 订阅
    unsigned fail = 0;
    bool got_msg = false;            // 是否已收到过主话题（未收到时不算延迟）
    double last_target_deg = 0.0;    // 最近一次话题上的主角度（打印用）
    rclcpp::Time last_msg_stamp;     // 最近一帧主消息的时间戳（算链路延迟）
    rclcpp::Time prev_cb_time = node->now();  // 前一帧回调时刻（差分算主角速度用）
    double prev_target_deg = 0.0;            // 前一帧主角度（差分算主角速度用）

    auto sub = node->create_subscription<motor_status_publisher::msg::MotorStatus>(
        "motor_status", rclcpp::QoS(10),
        [&](const motor_status_publisher::msg::MotorStatus::SharedPtr msg) {
            // 差分速度前馈：目标 = 主角度 + ff_gain_s×主角速度。
            // 从机位置环近似纯 P，匀速跟随存在 e_ss=v/Kp 的稳态滞后，用前馈把已知滞后
            // 提前补进指令（k≈1/Kp 时误差归 0）。k 过大/过小只留微小残差，不改变从机
            // 内部闭环稳定性。主角速度 = 相邻两帧话题角度差分；首帧/丢帧/角度回绕时
            // 前馈置 0（差分不可靠时退化为纯位置跟随）。
            double v_ff = 0.0;
            if (got_msg) {
                const double dt_s = (node->now() - prev_cb_time).seconds();
                const double d_deg = msg->angle_deg - prev_target_deg;
                if (dt_s > 1e-3 && dt_s < 0.5 && std::abs(d_deg) < 360.0) {
                    v_ff = d_deg / dt_s;
                }
            }
            prev_target_deg = msg->angle_deg;
            prev_cb_time = node->now();

            motor_can::MotorRunStatus rs;
            if (!motor.set_position(msg->angle_deg + ff_gain_s * v_ff, max_speed, &rs)) {
                if (++fail == 1 || fail % 100 == 0) {
                    RCLCPP_WARN(rclcpp::get_logger("motor_follower"),
                                "位置指令无回复（累计 %u 次），等待下帧", fail);
                }
            }
            last_target_deg = msg->angle_deg;
            last_msg_stamp = msg->header.stamp;
            got_msg = true;
        });

    // 1Hz 显示：主角度 / 从机当前角度 / 误差 / 链路延迟
    // 延迟 = 当前时间 - 主消息时间戳（发布节点 now() 打戳），正毫秒数，反映话题+CAN 链路耗时
    auto print_timer = node->create_wall_timer(
        kPrintPeriod, [&]() {
            double a2 = 0.0;
            if (!motor.read_angle(a2)) {
                return;
            }
            if (!got_msg) {
                RCLCPP_INFO(rclcpp::get_logger("motor_follower"),
                            "主 %.1f°  从 %.1f°  误差 %+.1f°（尚未收到主话题，指令失败 %u）",
                            last_target_deg, a2, last_target_deg - a2, fail);
                return;
            }
            const double latency_ms = (node->now() - last_msg_stamp).seconds() * 1000.0;
            RCLCPP_INFO(rclcpp::get_logger("motor_follower"),
                        "主 %.1f°  从 %.1f°  误差 %+.1f°  延迟 %.1fms（指令失败 %u）",
                        last_target_deg, a2, last_target_deg - a2, latency_ms, fail);
        });

    RCLCPP_INFO(rclcpp::get_logger("motor_follower"),
                "从机 %u 跟随开始（ifname=%s, max_speed=%u°/s, ff_gain=%.3fs），Ctrl-C 停止并锁闸",
                motor_id, ifname.c_str(), max_speed, ff_gain_s);

    // 退出清理：停止从机 + 锁闸（rclcpp 收到 SIGINT/SIGTERM 时触发）
    rclcpp::on_shutdown([&]() {
        motor.stop();
        motor.brake_lock();
        RCLCPP_INFO(rclcpp::get_logger("motor_follower"), "从机 %u 已停止并锁闸", motor_id);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
