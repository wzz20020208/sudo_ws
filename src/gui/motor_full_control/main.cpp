// gui/motor_full_control/main.cpp
// 全量电机控制前端：Qt5 Widgets 桌面程序。
// 单窗口 + 六个选项卡，汇总协议层全部已实现指令：三环/单圈/增量/力控控制、状态读取、
// PID 与加速度参数、编码器与零点、0x20 功能控制与 0xB6 主动回复、0x280 多机广播。
//
// 用法: ./motor_full_control [--ifname <can接口名>]
// 前置: sudo ip link set <ifname> type can bitrate 1000000 && up，并给电机供电。
// 安全: 会真实驱动电机运动，运行前请确认关节活动范围内无人；带抱闸电机运动前必须
//       先开闸；清多圈/复位/改 ID 等操作会弹确认框。

#include "full_control_window.hpp"

#include "motor_can/can_comm/can_comm.hpp"

#include <QApplication>
#include <QMessageBox>

#include <cstring>
#include <string>

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    std::string ifname = "can0";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--ifname") == 0 && i + 1 < argc) {
            ifname = argv[++i];
        } else {
            QMessageBox::critical(nullptr, "参数错误",
                                  "用法: ./motor_full_control [--ifname <can接口名>]");
            return 1;
        }
    }

    motor_can::CanConfig cfg;
    cfg.ifname = ifname;
    motor_can::CanComm comm;
    if (!comm.open(cfg)) {
        QMessageBox::critical(
            nullptr, "打开失败",
            QString("open(%1) 失败，请先 `sudo ip link set %1 type can bitrate 1000000 && up`")
                .arg(QString::fromStdString(ifname)));
        return 1;
    }

    FullControlWindow win(comm);
    win.show();
    const int rc = app.exec();
    // 窗口销毁后 comm 析构自动 close()（RAII）
    return rc;
}
