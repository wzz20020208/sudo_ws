// motor_status_publisher_node.cpp
// 发布电机实时状态：100Hz 定时读 0x9C，发布角度/转速/转矩电流到 /motor_status。
//
// 读失败策略：0x9C 单程通常 <2ms，reply_timeout 设 9ms 尽量不超 10ms 周期；
// 超时则发上次已知值（保持发布频率稳定），失败计数按节流告警不刷屏。
//
// 用法: ros2 run motor_status_publisher motor_status_publisher_node [--ifname can0] [--id 1]

#include "motor_can/can_comm/can_comm.hpp"
#include "motor_can/motor/motor.hpp"

#include "motor_status_publisher/msg/motor_status.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

constexpr auto kPublishPeriod = std::chrono::milliseconds(10);  // 100Hz
constexpr auto kReplyTimeout = std::chrono::milliseconds(9);    // 略小于周期，读失败不拖累频率

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 简单参数解析：--ifname <接口> --id <1..32>（与工程其它工具一致，不用 ROS 参数）
    std::string ifname = "can0";
    uint8_t motor_id = 1;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--ifname") == 0 && i + 1 < argc) {
            ifname = argv[++i];
        } else if (std::strcmp(argv[i], "--id") == 0 && i + 1 < argc) {
            motor_id = static_cast<uint8_t>(std::atoi(argv[++i]));
        }
    }

    motor_can::CanConfig cfg;
    cfg.ifname = ifname;
    motor_can::CanComm comm;
    if (!comm.open(cfg)) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_status_publisher"),
                     "open(%s) 失败，请先配置 CAN 接口", ifname.c_str());
        return 1;
    }

    // 只读句柄：home_on_init=false（不驱动电机），reply_timeout=9ms
    motor_can::Motor::Config mcfg;
    mcfg.home_on_init = false;
    mcfg.reply_timeout = kReplyTimeout;
    motor_can::Motor motor(comm, motor_id, mcfg);

    auto node = std::make_shared<rclcpp::Node>("motor_status_publisher");
    auto pub = node->create_publisher<motor_status_publisher::msg::MotorStatus>(
        "motor_status", rclcpp::QoS(10));

    motor_can::MotorRunStatus last{};
    bool valid = false;
    unsigned fail = 0;

    auto timer = node->create_wall_timer(kPublishPeriod, [&]() {
        motor_can::MotorRunStatus rs;
        if (motor.read_run_status(rs)) {
            last = rs;
            valid = true;
            fail = 0;
        } else if (++fail == 1 || fail % 100 == 0) {
            RCLCPP_WARN(rclcpp::get_logger("motor_status_publisher"),
                        "读取 0x9C 失败（累计 %u 次），发布上次已知值", fail);
        }
        if (!valid) {
            return;  // 从未读到过，不发布空数据
        }
        auto msg = motor_status_publisher::msg::MotorStatus();
        msg.header.stamp = node->now();
        msg.angle_deg = last.angle_deg;
        msg.speed_dps = last.speed_dps;
        msg.iq_a = last.iq_a;
        pub->publish(msg);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
