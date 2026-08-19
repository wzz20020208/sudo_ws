// include/motor_can/motor/motor.hpp
// 单机控制（Device 层）：通过传输层(CanComm) + 协议层(rh_protocol) 完成初始化与三环控制。
//
// 关键职责：
//  - 请求/回复过滤：发指令后循环 receive_by_id 直到回复命令字节 == 期望字节，
//    丢弃杂帧（CAN 总线共享时其他进程 / 其他电机的回复），解决里程碑 2 遗留的总线串扰。
//  - 三环控制：set_current（0xA1 转矩环）/ set_speed（0xA2 速度环）/
//    set_position（0xA4 位置环），成功返回 bool 并回读控制命令回复的运行状态。
//  - 初始化：构造时（home_on_init=true）自动开闸并物理归0（发位置指令到 0°，真实运动）。
//
// 线程安全：内部 mutex 串行化同一台电机的收发（保证过滤正确）；多线程并发调用安全，
// 不同电机实例互不阻塞（各自持锁）。
#pragma once

#include "motor_can/can_comm/can_comm.hpp"
#include "motor_can/protocol/rh_protocol.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace motor_can {

class Motor {
public:
    struct Config {
        uint16_t max_speed_dps = 30;                    // 位置环默认限速（°/s）
        uint8_t max_torque_pct = 30;                    // 速度环默认限扭（额定电流百分比）
        bool home_on_init = true;                       // 构造时物理归0（开闸 + 位置到 0°，真实运动）
        std::chrono::milliseconds reply_timeout{500};   // 单次回复等待上限
    };

    /// 构造要求 comm 已 open()。home_on_init=true 时自动开闸并归0（会真实驱动电机）。
    Motor(CanComm& comm, uint8_t id);              ///< 默认配置（构造即归0）
    Motor(CanComm& comm, uint8_t id, const Config& config);
    ~Motor();  // 停止录制并 join 录制线程，避免线程访问已析构成员

    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    uint8_t id() const noexcept { return id_; }

    // ---- 三环控制：成功返回 true，并通过 out 回读控制命令回复的运行状态 ----

    /// 转矩环 0xA1：目标电流（A）。抱闸电机使用前需先 brake_release()。
    bool set_current(double current_a, MotorRunStatus* out = nullptr);

    /// 速度环 0xA2：目标转速（°/s），限扭用 config.max_torque_pct。
    bool set_speed(double speed_dps, MotorRunStatus* out = nullptr);

    /// 速度环 0xA2：目标转速（°/s）+ 限扭（额定电流百分比）。
    bool set_speed(double speed_dps, uint8_t max_torque_pct, MotorRunStatus* out = nullptr);

    /// 位置环 0xA4：目标多圈角度（°），限速用 config.max_speed_dps。
    bool set_position(double angle_deg, MotorRunStatus* out = nullptr);

    /// 位置环 0xA4：目标多圈角度（°）+ 限速（°/s）。
    bool set_position(double angle_deg, uint16_t max_speed_dps, MotorRunStatus* out = nullptr);

    // ---- 初始化 / 制动辅助 ----

    /// 物理归0：0x77 开闸 → 0xA4 位置指令到 0°（以当前零点为参考）。
    bool home();

    bool brake_release();  ///< 0x77 抱闸释放
    bool brake_lock();     ///< 0x78 抱闸锁死（带抱闸电机应先在停止状态）
    bool stop();           ///< 0x81 停止电机并保持不动
    bool off();            ///< 0x80 关闭电机输出，清除运行状态

    // ---- 参数配置 / 零点（均走请求/回复过滤）----

    /// 0x30 读指定环 PID 参数。value：该环参数。
    bool read_pid(PidIndex index, float& value);

    /// 0x31 写指定环 PID 参数到 RAM（掉电不保存）。value：目标参数。
    bool write_pid_ram(PidIndex index, double value);

    /// 0x32 写指定环 PID 参数到 ROM（掉电保存）。value：目标参数。
    bool write_pid_rom(PidIndex index, double value);

    /// 0x64 将当前编码器位置写为零点（写 ROM）。new_offset：电机回显的新零偏（脉冲），
    /// 需 0x76 系统复位后新零点才生效。
    bool set_zero_point(int32_t& new_offset);

    /// 0x20 索引 0x01 清除电机多圈值：清零多圈计数、更新零点并保存，重启后生效。
    /// 走请求/回复过滤（等原帧回显），成功返回 true。
    bool clear_multi_turn();

    /// 0xB6 设置主动回复：使能/关闭指定指令按固定间隔主动上报（无回复，fire-and-forget）。
    /// report_cmd：要主动上报的指令（0x60/0x61/0x62/0x92/0x9A/0x9C/0x9D/0x9E）；
    /// enable：true=使能，false=关闭；interval_10ms：上报间隔，10ms/LSB。
    /// 返回发送是否成功（无回复可等，生效与否需观察总线主动帧）。
    bool set_active_report(uint8_t report_cmd, bool enable, uint16_t interval_10ms);

    /// 0x20 索引 0x02 CANID 滤波器使能（存 FLASH）。true=使能（提高收发效率），
    /// false=失能（0x280 多电机广播前置条件，广播时须先失能）。
    bool set_can_id_filter(bool enable);

    /// 0x20 索引 0x03 错误状态发送使能。true=出错后主动向总线发 0x9A（100ms 周期），
    /// false=失能（错误状态消失后停止发送）。
    bool set_error_report(bool enable);

    /// 0x20 索引 0x04 多圈值掉电保存使能（重启后生效）。true=掉电前保存当前多圈值，
    /// false=失能，系统默认单圈模式。
    bool set_multi_turn_power_save(bool enable);

    /// 0x20 索引 0x06/0x07 设置位置运行最大正/负角度（存 ROM 立即生效，两条连续写）。
    /// 单位按 0.01°/LSB 打包（手册未注明单位，待真机确认）。执行中不可并行发位置指令。
    bool set_position_limits(double max_pos_deg, double max_neg_deg);

    /// 0xB3 设置通讯中断保护时间（ms，写 ROM，0=关闭保护）。通讯中断超过设定时间
    /// 会切断输出并锁死抱闸；注意避免在电机刚启动以及运动时写入。
    bool set_com_protect_time(uint32_t time_ms);

    // ---- 状态读取 ----

    bool read_status(MotorStatus& out);         ///< 0x9A 温度/电压/抱闸/错误
    bool read_run_status(MotorRunStatus& out);  ///< 0x9C 转速/电流/角度
    bool read_angle(double& angle_deg);         ///< 0x92 多圈绝对角度
    bool read_status3(MotorStatus3& out);       ///< 0x9D 温度 + 三相相电流 iA/iB/iC
    bool read_single_encoder(SingleEncoder& enc);  ///< 0x90 单圈编码器 encoder/raw/offset（直驱用）
    bool read_run_time(uint32_t& time_ms);      ///< 0xB1 系统运行时间（ms）
    bool read_version_date(uint32_t& date);     ///< 0xB2 软件版本日期（yyyymmdd）
    bool read_motor_model(char model[8]);       ///< 0xB5 电机型号（7 个 ASCII）

    // ---- 数据录制（独立线程周期轮询写入 CSV，可用于无人值守 / 无 GUI 场景）----

    /// 开始录制：打开 path（truncate）写 CSV 表头并启动录制线程，线程每 interval 读
    /// 0x9A+0x9C 记一行（列：motor_id,t_s,voltage_v,speed_dps,angle_deg,iq_a，角度为 0x9C
    /// 原始值，回绕不在此解包）。已在录制时返回 false（需先 stop_record）；
    /// 文件打开失败返回 false。录制读取与其它操作共用内部锁，线程安全。
    bool start_record(const std::string& path,
                      std::chrono::milliseconds interval = std::chrono::milliseconds(100));

    /// 停止录制：置停止标志、join 录制线程并关闭文件；已录数据完整保留在磁盘上。
    /// 幂等（未录制时为 no-op）。
    void stop_record();

    /// 是否正在录制（供调用方刷新按钮/状态）。
    bool is_recording() const noexcept { return record_running_; }

    // ---- 单圈位置控制（0xA6，直驱用）----

    /// 0xA6 单圈位置闭环：direction=0 顺时针 / 1 逆时针，max_speed_dps 最大转速（°/s），
    /// angle_deg 目标单圈角度（0°~359.99°）。回复布局同 0x9C，通过 out 回读。
    bool set_single_angle_position(uint8_t direction, uint16_t max_speed_dps, double angle_deg,
                                   MotorRunStatus* out = nullptr);

private:
    /// 发一帧并等待命令字节 == expected_cmd 的回复；杂帧直接丢弃，
    /// 总超时用 steady_clock 递减剩余时间，超时或接口关闭返回 false。
    bool request(const CanFrame& frame, uint8_t expected_cmd, CanFrame& reply);

    /// 锁内执行：request() + （可选）decode_run_status 回读。
    bool control(const CanFrame& frame, uint8_t expected_cmd, MotorRunStatus* out);

    /// 0x20 功能控制写（锁内调用）：发 index+value 并等命令字节 + 索引匹配的回显。
    bool function_control(FunctionIndex index, uint32_t value);

    void record_loop();  // 录制线程体：每 interval 读 0x9A+0x9C，成功后写一行 CSV

    CanComm& comm_;
    uint8_t id_;
    Config config_;
    mutable std::mutex mutex_;

    // ---- 录制（独立线程；record_file_ 仅录制线程写，start/stop 通过 join 同步）----
    std::thread record_thread_;                  // 录制线程（start_record 启动，stop_record/析构 join）
    std::atomic<bool> record_running_{false};    // 正在录制（跨线程标志）
    std::atomic<bool> record_stop_{false};       // 请求停止（录制循环退出条件）
    std::chrono::steady_clock::time_point record_start_;  // 录制开始时刻（t_s 相对基准）
    std::chrono::milliseconds record_interval_{100};      // 录制轮询周期
    std::ofstream record_file_;                  // CSV 输出流
};

}  // namespace motor_can
