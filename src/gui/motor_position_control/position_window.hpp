// gui/motor_position_control/position_window.hpp
// 位置闭环控制窗口：选择电机 ID，轮询 0x92（当前多圈角度）+ 0x9C（转速/电流），
// 可手动开闸（0x77）、发绝对位置指令到目标角度 / 回0点（0xA4）、停止（0x81）、锁闸（0x78）。
//
// 设计（刻意保持简单）：
//  - 不用 QThread/自定义信号：QTimer 在主线程轮询，connect 接 lambda，
//    本类无 Q_OBJECT，也不需要 AUTOMOC。
//  - 控制动作串行收发，执行期间暂停轮询 timer，避免 0x92/0x9C 回复混入操作队列。
//  - 抱闸手动控制：运动前需先点「开闸」（0x77）；锁闸自动先发 0x81 停止再 0x78。
//  - 多电机顺序：逗号分隔角度列表（如 180,360,720），第 i 个角度发 0xA4 到 ID i+1，
//    逐个发送并等每台回复；转速复用上方「最大转速」输入框。
#pragma once

#include "motor_can/protocol/rh_protocol.hpp"

#include <QWidget>

#include <cstdint>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTimer;

namespace motor_can {
class CanComm;
struct CanFrame;
}

class PositionWindow : public QWidget {
public:
    explicit PositionWindow(motor_can::CanComm& comm);
    ~PositionWindow() override;

private:
    // 发一条单字节读取命令并等待电机回复；成功返回 true 并填充 reply
    bool query(uint8_t cmd, motor_can::CanFrame& reply);

    // 发送任意协议帧并等待本电机回复
    bool send_and_receive(const motor_can::CanFrame& frame, motor_can::CanFrame& reply);

    void poll();  // 轮询 0x92 + 0x9C，刷新实时区

    // 发绝对位置指令（0xA4）到 angle_deg；回0点即 angle_deg=0
    void run_to(double angle_deg);
    void brake_release();  // 0x77
    void motor_stop();     // 0x81
    void brake_lock();     // 0x81 + 0x78（先停再锁）

    // 多电机顺序：解析逗号分隔角度列表，第 i 个角度发 0xA4 到 ID i+1，逐个等回复
    void send_sequence();

    motor_can::CanComm& comm_;
    int motor_id_ = 1;  // 当前选中的电机 ID

    QSpinBox* id_spin_ = nullptr;
    QLabel* status_label_ = nullptr;
    QTimer* timer_ = nullptr;

    // 目标位置
    QDoubleSpinBox* target_box_ = nullptr;
    QSpinBox* speed_box_ = nullptr;

    // 实时区
    QLabel* angle_label_ = nullptr;
    QLabel* speed_label_ = nullptr;
    QLabel* current_label_ = nullptr;

    // 多电机顺序（0xA4）
    QLineEdit* seq_angles_edit_ = nullptr;
    QPushButton* seq_send_btn_ = nullptr;
    QLabel* seq_result_label_ = nullptr;
};
