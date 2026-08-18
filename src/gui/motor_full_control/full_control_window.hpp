// gui/motor_full_control/full_control_window.hpp
// 全量电机控制窗口：单窗口 + 六个选项卡，汇总协议层全部已实现指令
// （保留现有 monitor / position 两个窗口不动，本窗口是独立全量版）。
//
// 设计：
//  - 不用 QThread/自定义信号：QTimer 主线程轮询 0x9A + 0x9C，connect 接 lambda，
//    本类无 Q_OBJECT，也不需要 AUTOMOC（与现有两个窗口一致）。
//  - 单机操作优先走 Device 层 Motor 过滤接口（发指令后按命令字节过滤杂帧），
//    覆盖 Device 层已暴露的方法；Device 未暴露的指令（0xA8/0xA9/0x70/0x94、
//    0x60~0x63、0x42/0x43、0x76）走 raw 协议 + CanComm 直接收发。
//  - Motor 配置 home_on_init=false（只读句柄，不驱动电机）+ reply_timeout=100ms。
//  - 所有带副作用的操作（清多圈/复位/关输出/改 ID/写零偏/掉电保存等）一律弹
//    QMessageBox 确认，默认 No；操作期间暂停轮询 timer，避免杂帧串扰。
//  - 0xB4 波特率不暴露（改完立即断联，会让本窗口自己的连接失效）。
#pragma once

#include "motor_can/motor/can_id.hpp"
#include "motor_can/motor/motor.hpp"
#include "motor_can/protocol/rh_protocol.hpp"

#include <QWidget>

#include <cstdint>
#include <memory>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTabWidget;
class QTimer;

namespace motor_can {
class CanComm;
}

class FullControlWindow : public QWidget {
public:
    explicit FullControlWindow(motor_can::CanComm& comm);
    ~FullControlWindow() override;

private:
    // ---- Tab 构建（构造器调用，只建布局 + connect，行为在下方同名方法）----

    QWidget* build_tab_control();
    QWidget* build_tab_status();
    QWidget* build_tab_params();
    QWidget* build_tab_encoder();
    QWidget* build_tab_system();
    QWidget* build_tab_multi();

    // ---- 通用 ----

    void poll();  // 轮询 0x9A + 0x9C，刷新顶部状态与 Tab1 实时区
    void rebuild_motor();  // 为当前 motor_id_ 重建只读 Motor 句柄

    // raw 协议查询：发一帧到 0x140+id，等 0x240+id 回复（100ms）。回复命令字节不符
    // 由各 decode_* 判 false。操作方法应在 timer_->stop() 期间调用。
    bool query(uint8_t cmd, motor_can::CanFrame& reply);
    bool send_and_receive(const motor_can::CanFrame& frame, motor_can::CanFrame& reply);

    // ---- Tab1 控制 ----

    void set_current();         // 0xA1 转矩环
    void set_speed();           // 0xA2 速度环
    void set_position();        // 0xA4 绝对位置环
    void set_single_angle();    // 0xA6 单圈位置（直驱用）
    void set_increment();       // 0xA8 增量位置（raw）
    void set_force_position();  // 0xA9 力控位置（raw）
    void brake_release();       // 0x77
    void brake_lock();          // 先 0x81 停止再 0x78
    void motor_stop();          // 0x81
    void motor_off();           // 0x80
    void motor_home();          // 物理归0：0x77 → 0xA4 到 0°

    // ---- Tab2 状态（逐行读取）----

    void read_status1();        // 0x9A
    void read_status2();        // 0x9C
    void read_status3();        // 0x9D 三相相电流
    void read_angle_now();      // 0x92 多圈角度
    void read_single_angle();   // 0x94 单圈角度（raw）
    void read_single_encoder(); // 0x90 单圈编码器
    void read_mode();           // 0x70 运行模式（raw）
    void read_run_time();       // 0xB1 运行时间
    void read_version_date();   // 0xB2 版本日期
    void read_motor_model();    // 0xB5 型号

    // ---- Tab3 参数 ----

    void read_pid_all();                       // 0x30 读 7 个环参数填框
    void write_pid_all(motor_can::RhCmd cmd);  // 0x31 RAM / 0x32 ROM 写可配置环
    void read_accel_all();                     // 0x42 读 4 项加速度
    void write_accel_all();                    // 0x43 写 4 项加速度（RAM+ROM）

    // ---- Tab4 编码器 / 零点 ----

    void read_encoders();        // 0x60/0x61/0x62 一次读多圈位置/原始位置/零偏
    void write_encoder_offset(); // 0x63 写零偏（raw）
    void set_zero_point();       // 0x64 记当前点为零点
    void clear_multi_turn();     // 0x20/0x01 清除多圈值
    void apply_power_save();     // 0x20/0x04 多圈掉电保存开关

    // ---- Tab5 系统 ----

    void apply_canid_filter();   // 0x20/0x02 CANID 滤波器开关
    void apply_error_report();   // 0x20/0x03 错误上报开关
    void change_can_id();        // 0x20/0x05 修改 CAN ID（扫描→确认→写）
    void send_active_report();   // 0xB6 主动回复（无回复）
    void write_protect_time();   // 0xB3 通讯中断保护时间
    void write_position_limits();// 0x20/0x06/0x07 位置运行限位
    void reset_motor();          // 0x76 系统复位（无回复）

    // ---- Tab6 多机 ----

    void send_sequence();        // 逗号角度列表 → ID 1..n 顺序发 0xA4
    void broadcast_stop();       // 0x280 + 0x81
    void broadcast_off();        // 0x280 + 0x80
    void broadcast_position();   // 0x280 + 0xA4
    void scan_online();          // enumerate_can_ids 扫描在线 ID

    motor_can::CanComm& comm_;
    int motor_id_ = 1;  // 当前选中的电机 ID

    motor_can::Motor::Config mcfg_;          // 只读配置：home_on_init=false, reply_timeout=100ms
    std::unique_ptr<motor_can::Motor> motor_;  // 过滤读接口的持有者

    // ---- 顶部条 ----
    QSpinBox* id_spin_ = nullptr;
    QLabel* status_label_ = nullptr;
    QPushButton* scan_btn_ = nullptr;
    QLabel* online_label_ = nullptr;
    QTimer* timer_ = nullptr;

    // ---- Tab1 控制 ----
    QDoubleSpinBox* cur_target_box_ = nullptr;    // 0xA1 目标电流（A）
    QDoubleSpinBox* spd_target_box_ = nullptr;    // 0xA2 目标转速（°/s）
    QSpinBox* spd_torque_box_ = nullptr;          // 0xA2 限扭（额定电流百分比）
    QDoubleSpinBox* pos_target_box_ = nullptr;    // 0xA4 目标角度（°）
    QSpinBox* pos_speed_box_ = nullptr;           // 0xA4 最大转速（°/s）
    QComboBox* single_dir_box_ = nullptr;         // 0xA6 方向 0=顺/1=逆
    QSpinBox* single_speed_box_ = nullptr;        // 0xA6 最大转速
    QDoubleSpinBox* single_angle_box_ = nullptr;  // 0xA6 目标单圈角度
    QDoubleSpinBox* inc_delta_box_ = nullptr;     // 0xA8 角度增量（°）
    QSpinBox* inc_speed_box_ = nullptr;           // 0xA8 最大转速
    QDoubleSpinBox* force_target_box_ = nullptr;  // 0xA9 目标角度（°）
    QSpinBox* force_speed_box_ = nullptr;         // 0xA9 最大转速
    QSpinBox* force_torque_box_ = nullptr;        // 0xA9 最大扭矩（额定电流百分比）

    QLabel* rt_angle_label_ = nullptr;   // 实时角度（0x9C）
    QLabel* rt_speed_label_ = nullptr;   // 实时转速
    QLabel* rt_current_label_ = nullptr; // 实时转矩电流
    QLabel* rt_temp_label_ = nullptr;    // 实时电机温度

    // ---- Tab2 状态 ----
    QLabel* st1_result_ = nullptr;
    QLabel* st2_result_ = nullptr;
    QLabel* st3_result_ = nullptr;
    QLabel* ang_result_ = nullptr;
    QLabel* sang_result_ = nullptr;
    QLabel* enc_result_ = nullptr;
    QLabel* mode_result_ = nullptr;
    QLabel* time_result_ = nullptr;
    QLabel* ver_result_ = nullptr;
    QLabel* model_result_ = nullptr;

    // ---- Tab3 参数 ----
    QDoubleSpinBox* cur_kp_box_ = nullptr;
    QDoubleSpinBox* cur_ki_box_ = nullptr;
    QDoubleSpinBox* spd_kp_box_ = nullptr;
    QDoubleSpinBox* spd_ki_box_ = nullptr;
    QDoubleSpinBox* pos_kp_box_ = nullptr;
    QDoubleSpinBox* pos_ki_box_ = nullptr;
    QDoubleSpinBox* pos_kd_box_ = nullptr;
    QDoubleSpinBox* pos_accel_box_ = nullptr;  // 位置规划加速度
    QDoubleSpinBox* pos_decel_box_ = nullptr;  // 位置规划减速度
    QDoubleSpinBox* spd_accel_box_ = nullptr;  // 速度规划加速度
    QDoubleSpinBox* spd_decel_box_ = nullptr;  // 速度规划减速度
    QLabel* param_result_label_ = nullptr;

    // ---- Tab4 编码器 / 零点 ----
    QLabel* enc_pos_label_ = nullptr;   // 0x60 多圈位置（脉冲）
    QLabel* enc_raw_label_ = nullptr;   // 0x61 原始位置（脉冲）
    QLabel* enc_offset_label_ = nullptr; // 0x62 零偏（脉冲）
    QSpinBox* enc_offset_box_ = nullptr;  // 0x63 新零偏输入（脉冲）
    QCheckBox* power_save_check_ = nullptr; // 0x20/0x04 掉电保存多圈
    QLabel* enc_result_label_ = nullptr;

    // ---- Tab5 系统 ----
    QCheckBox* filter_check_ = nullptr;       // 0x20/0x02 CANID 滤波器
    QCheckBox* error_report_check_ = nullptr; // 0x20/0x03 错误上报
    QSpinBox* new_id_box_ = nullptr;          // 0x20/0x05 新 CAN ID
    QComboBox* report_cmd_box_ = nullptr;     // 0xB6 要主动上报的指令
    QCheckBox* report_enable_check_ = nullptr;// 0xB6 使能/关闭
    QSpinBox* report_interval_box_ = nullptr; // 0xB6 上报间隔（10ms/LSB）
    QSpinBox* protect_time_box_ = nullptr;    // 0xB3 中断保护时间（ms）
    QDoubleSpinBox* limit_pos_box_ = nullptr; // 0x20/0x06 最大正角度
    QDoubleSpinBox* limit_neg_box_ = nullptr; // 0x20/0x07 最大负角度
    QLabel* sys_result_label_ = nullptr;

    // ---- Tab6 多机 ----
    QLineEdit* seq_angles_edit_ = nullptr;  // 逗号分隔角度列表
    QSpinBox* seq_speed_box_ = nullptr;     // 顺序发送转速
    QDoubleSpinBox* bcast_angle_box_ = nullptr; // 广播位置目标角度
    QSpinBox* bcast_speed_box_ = nullptr;       // 广播位置转速
    QLabel* multi_result_label_ = nullptr;
};
