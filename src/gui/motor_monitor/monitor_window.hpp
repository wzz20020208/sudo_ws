// gui/motor_monitor/monitor_window.hpp
// 电机监测窗口：选择电机 ID，轮询 0x9A（温度/电压/抱闸/错误）+ 0x9C（转速/电流/角度），
// 外加三环 PID 配置（0x30/0x31/0x32）与多圈零点记录（0x64）。
//
// 设计（刻意保持简单）：
//  - 不用 QThread/自定义信号：QTimer 在主线程轮询，connect 接 lambda，
//    本类无 Q_OBJECT，也不需要 AUTOMOC。
//  - 读写全部走 Device 层 Motor 的过滤接口（read_status / read_run_status / read_pid 等）：
//    发指令后循环收直到命令字节匹配，杂帧自动丢弃——监控进程可与另一台控制程序
//    同时运行（各自过滤），监控本身只读、绝不发运动指令。
//  - Motor 配置 home_on_init=false（只读句柄，不驱动电机）+ reply_timeout=100ms（界面灵敏）。
//  - 切换电机 ID 时重建 Motor（仅换只读句柄）。
#pragma once

#include "motor_can/motor/can_id.hpp"
#include "motor_can/motor/motor.hpp"
#include "motor_can/protocol/rh_protocol.hpp"

#include <QWidget>

#include <cstdint>
#include <memory>

class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSpinBox;
class QTimer;

namespace motor_can {
class CanComm;
}

class MonitorWindow : public QWidget {
public:
    explicit MonitorWindow(motor_can::CanComm& comm);
    ~MonitorWindow() override;

private:
    void poll();  // 轮询 0x9A + 0x9C，刷新界面

    // 三环 PID：读取 7 个参数填框；cmd 为 PidWriteRam(0x31) 或 PidWriteRom(0x32) 时写 5 个可配置参数
    void read_pid_all();
    void write_pid_all(motor_can::RhCmd cmd);

    // 记录当前编码器位置为多圈零点（0x64），需 0x76 复位后生效
    void set_zero_point();

    // 修改 CAN ID（0x20 索引 0x05，按地址写）：先扫描在线 ID，目标 ID 在线才写
    void change_can_id();

    // 为当前 motor_id_ 重建只读 Motor 句柄（切换电机时调用）
    void rebuild_motor();

    motor_can::CanComm& comm_;
    int motor_id_ = 1;  // 当前选中的电机 ID

    motor_can::Motor::Config mcfg_;                    // 只读配置：home_on_init=false, reply_timeout=100ms
    std::unique_ptr<motor_can::Motor> motor_;          // 过滤读接口的持有者

    QSpinBox* id_spin_ = nullptr;
    QLabel* status_label_ = nullptr;
    QTimer* timer_ = nullptr;

    // 0x9A 状态
    QLabel* temp_label_ = nullptr;
    QLabel* mos_label_ = nullptr;
    QLabel* volt_label_ = nullptr;
    QLabel* brake_label_ = nullptr;
    QLabel* error_label_ = nullptr;

    // 0x9C 运行数据
    QLabel* speed_label_ = nullptr;
    QLabel* current_label_ = nullptr;
    QLabel* angle_label_ = nullptr;

    // 三环 PID（电流环只读）
    QDoubleSpinBox* cur_kp_box_ = nullptr;
    QDoubleSpinBox* cur_ki_box_ = nullptr;
    QDoubleSpinBox* spd_kp_box_ = nullptr;
    QDoubleSpinBox* spd_ki_box_ = nullptr;
    QDoubleSpinBox* pos_kp_box_ = nullptr;
    QDoubleSpinBox* pos_ki_box_ = nullptr;
    QDoubleSpinBox* pos_kd_box_ = nullptr;

    // 多圈零点
    QLabel* zero_result_label_ = nullptr;

    // 修改 CAN ID（0x20 索引 0x05）
    QSpinBox* new_id_spin_ = nullptr;
    QPushButton* change_id_btn_ = nullptr;
    QLabel* canid_result_label_ = nullptr;
};
