// gui/motor_monitor/main.cpp
// 电机状态监测前端：Qt5 Widgets 桌面程序。
// 用现有 CanComm::receive_by_id 轮询所选电机的 0x9A 状态（温度/电压/抱闸/错误），
// 只读监测，不发任何运动指令。
//
// 用法: ./motor_monitor [--ifname <can接口名>]
// 前置: sudo ip link set <ifname> type can bitrate 1000000 && up，并给电机供电。

#include "monitor_window.hpp"

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
                                  "用法: ./motor_monitor [--ifname <can接口名>]");
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

    MonitorWindow win(comm);
    win.show();
    const int rc = app.exec();
    // 窗口销毁后 comm 析构自动 close()（RAII）
    return rc;
}
