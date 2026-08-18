// gui/motor_position_control/main.cpp
// 位置闭环控制前端：Qt5 Widgets 桌面程序。
// 轮询所选电机的 0x92 当前角度 + 0x9C 运行数据，可发绝对位置指令（0xA4）到目标角度 /
// 回0点，手动开闸（0x77）/ 停止（0x81）/ 锁闸（0x78）。
//
// 用法: ./motor_position_control [--ifname <can接口名>]
// 前置: sudo ip link set <ifname> type can bitrate 1000000 && up，并给电机供电。
// 安全: 会真实驱动电机运动，运行前请确认关节活动范围内无人；
//       带抱闸电机运动前必须先点「开闸」，结束建议「停止」+「锁闸」。

#include "position_window.hpp"

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
                                  "用法: ./motor_position_control [--ifname <can接口名>]");
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

    PositionWindow win(comm);
    win.show();
    const int rc = app.exec();
    return rc;
}
