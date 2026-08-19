// src/motor/motor.cpp
// Motor 单机控制实现：请求/回复过滤 + 三环控制 + 初始化归0。

#include "motor_can/motor/motor.hpp"

#include "motor_can/common/log.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>

namespace motor_can {

// 委托默认配置构造（home_on_init=true：构造即物理归0）
Motor::Motor(CanComm& comm, uint8_t id) : Motor(comm, id, Config()) {}

// 析构：先停录制（join 录制线程，避免线程在成员析构后仍访问 this），其它资源随成员析构
Motor::~Motor() {
    stop_record();
}

// 带配置构造：保存 comm 引用/ID/配置；home_on_init=true 时物理归0（开闸 + 位置到 0°）
Motor::Motor(CanComm& comm, uint8_t id, const Config& config)
    : comm_(comm), id_(id), config_(config) {
    if (config_.home_on_init) {
        MC_LOG_INFO("Motor(id=%u) 构造：物理归0（开闸 + 位置到 0°）", id_);
        if (!home()) {
            MC_LOG_ERROR("Motor(id=%u) 归0 失败，请检查通讯与电机状态", id_);
        }
    }
}

// 请求/回复过滤：发帧后循环收 0x240+id，命令字节匹配才收下，杂帧丢弃；总超时递减
bool Motor::request(const CanFrame& frame, uint8_t expected_cmd, CanFrame& reply) {
    if (!comm_.send(frame)) {
        return false;
    }
    // 总超时：从发出时刻起递减剩余时间，避免多个杂帧把超时无限拖长
    const auto deadline = std::chrono::steady_clock::now() + config_.reply_timeout;
    while (true) {
        const auto remain = deadline - std::chrono::steady_clock::now();
        if (remain <= std::chrono::milliseconds::zero()) {
            return false;  // 总超时
        }
        if (!comm_.receive_by_id(0x240u + id_, reply,
                                 std::chrono::duration_cast<std::chrono::milliseconds>(remain))) {
            return false;  // 单次等待超时或接口已关闭
        }
        if (reply.data[0] == expected_cmd) {
            return true;  // 命令字节匹配，收下
        }
        // 杂帧（其他进程/电机的回复）：丢弃，继续收
    }
}

// 锁内发控制帧（三环共用）：request 校验命令字节，out 非空时回读运行状态
bool Motor::control(const CanFrame& frame, uint8_t expected_cmd, MotorRunStatus* out) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(frame, expected_cmd, reply)) {
        return false;
    }
    if (out == nullptr) {
        return true;  // request 已校验命令字节
    }
    return decode_run_status(reply, *out);
}

// 转矩环 0xA1：目标电流（A）；抱闸电机使用前需先 brake_release
bool Motor::set_current(double current_a, MotorRunStatus* out) {
    return control(encode_torque(id_, current_a), static_cast<uint8_t>(RhCmd::Torque), out);
}

// 速度环 0xA2：目标转速（°/s），限扭用配置默认
bool Motor::set_speed(double speed_dps, MotorRunStatus* out) {
    return set_speed(speed_dps, config_.max_torque_pct, out);
}

// 速度环 0xA2：目标转速（°/s）+ 指定限扭（额定电流百分比）
bool Motor::set_speed(double speed_dps, uint8_t max_torque_pct, MotorRunStatus* out) {
    return control(encode_speed(id_, speed_dps, max_torque_pct),
                   static_cast<uint8_t>(RhCmd::Speed), out);
}

// 位置环 0xA4：目标多圈角度（°），限速用配置默认
bool Motor::set_position(double angle_deg, MotorRunStatus* out) {
    return set_position(angle_deg, config_.max_speed_dps, out);
}

// 位置环 0xA4：目标多圈角度（°）+ 指定限速（°/s）
bool Motor::set_position(double angle_deg, uint16_t max_speed_dps, MotorRunStatus* out) {
    return control(encode_position(id_, angle_deg, max_speed_dps),
                   static_cast<uint8_t>(RhCmd::Position), out);
}

// 物理归0：开闸后位置到 0°（0°=当前编码器零点，真实运动）
bool Motor::home() {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    // 带抱闸电机运动前必须开闸
    if (!request(encode_command(id_, RhCmd::BrakeRelease),
                 static_cast<uint8_t>(RhCmd::BrakeRelease), reply)) {
        return false;
    }
    // 位置指令到 0°（0° = 当前编码器零点）
    if (!request(encode_position(id_, 0.0, config_.max_speed_dps),
                 static_cast<uint8_t>(RhCmd::Position), reply)) {
        return false;
    }
    return true;
}

// 0x77 抱闸释放：带抱闸电机运动前必须调用（回复为原帧回显）
bool Motor::brake_release() {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    return request(encode_command(id_, RhCmd::BrakeRelease),
                   static_cast<uint8_t>(RhCmd::BrakeRelease), reply);
}

// 0x78 抱闸锁死：停止后抱紧（回复为原帧回显）
bool Motor::brake_lock() {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    return request(encode_command(id_, RhCmd::BrakeLock),
                   static_cast<uint8_t>(RhCmd::BrakeLock), reply);
}

// 0x81 停止电机并保持不动（回复为原帧回显）
bool Motor::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    return request(encode_command(id_, RhCmd::MotorStop),
                   static_cast<uint8_t>(RhCmd::MotorStop), reply);
}

// 0x80 关闭电机输出，清除运行状态（回复为原帧回显）
bool Motor::off() {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    return request(encode_command(id_, RhCmd::MotorOff),
                   static_cast<uint8_t>(RhCmd::MotorOff), reply);
}

// 0x30 读指定环 PID：value 为 float32（IEEE754）解码结果
bool Motor::read_pid(PidIndex index, float& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_read_pid(id_, index), static_cast<uint8_t>(RhCmd::PidRead), reply)) {
        return false;
    }
    return decode_pid(reply, value);
}

// 0x31 写指定环 PID 到 RAM（掉电不保存），回显验证已写入
bool Motor::write_pid_ram(PidIndex index, double value) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_write_pid_ram(id_, index, value),
                 static_cast<uint8_t>(RhCmd::PidWriteRam), reply)) {
        return false;
    }
    float echo = 0.0f;
    return decode_pid(reply, echo);  // 写命令回显同 PID 布局，验证已写入
}

// 0x32 写指定环 PID 到 ROM（掉电保存），回显验证已写入
bool Motor::write_pid_rom(PidIndex index, double value) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_write_pid_rom(id_, index, value),
                 static_cast<uint8_t>(RhCmd::PidWriteRom), reply)) {
        return false;
    }
    float echo = 0.0f;
    return decode_pid(reply, echo);
}

// 0x64 当前编码器位置写为零点（写 ROM）；new_offset 回显新零偏（脉冲），需 0x76 复位生效
bool Motor::set_zero_point(int32_t& new_offset) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::WriteCurrentZero),
                 static_cast<uint8_t>(RhCmd::WriteCurrentZero), reply)) {
        return false;
    }
    return decode_encoder_position(reply, new_offset);  // 0x64 回显新零偏
}

// 0x20 索引 0x01 清除多圈值：清多圈、更新零点并保存，重启后生效
bool Motor::clear_multi_turn() {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    // 0x20 索引 0x01 回复为原帧回显，命令字节校验到 FunctionControl 即可
    if (!request(encode_clear_multi_turn(id_), static_cast<uint8_t>(RhCmd::FunctionControl),
                 reply)) {
        return false;
    }
    return decode_clear_multi_turn(reply);  // 再校验功能索引为 0x01
}

// 0xB6 配置主动上报（无回复）：report_cmd 按 interval_10ms（10ms/LSB）周期主动上报，生效与否看总线帧
bool Motor::set_active_report(uint8_t report_cmd, bool enable, uint16_t interval_10ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 0xB6 无回复，发送成功即返回；生效与否靠观察总线上的主动上报帧
    return comm_.send(encode_active_report(id_, report_cmd, enable, interval_10ms));
}

// 0x20 通用功能索引写（锁内调用）：发 index+value，命令字节与索引均匹配的回显才收下
bool Motor::function_control(FunctionIndex index, uint32_t value) {
    CanFrame reply;
    if (!request(encode_function_control(id_, index, value),
                 static_cast<uint8_t>(RhCmd::FunctionControl), reply)) {
        return false;
    }
    uint32_t echo = 0;
    return decode_function_control_echo(reply, index, echo);
}

// 0x20 索引 0x02 CANID 滤波器使能（存 FLASH）；0x280 多机广播前置需失能
bool Motor::set_can_id_filter(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    return function_control(FunctionIndex::CanIdFilter, enable ? 1 : 0);
}

// 0x20 索引 0x03 错误上报使能：出错后主动发 0x9A（100ms 周期）
bool Motor::set_error_report(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    return function_control(FunctionIndex::ErrorReport, enable ? 1 : 0);
}

// 0x20 索引 0x04 多圈值掉电保存使能（重启后生效）
bool Motor::set_multi_turn_power_save(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    return function_control(FunctionIndex::MultiTurnPowerSave, enable ? 1 : 0);
}

// 0x20 索引 0x06/0x07 位置正/负限位（存 ROM 立即生效）；按 0.01°/LSB 打包，待真机确认
bool Motor::set_position_limits(double max_pos_deg, double max_neg_deg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 两条索引连续写；单位按 0.01°/LSB（手册未注明，待真机确认）
    if (!function_control(FunctionIndex::MaxPosAngle,
                          static_cast<uint32_t>(std::lround(max_pos_deg * 100.0)))) {
        return false;
    }
    return function_control(FunctionIndex::MaxNegAngle,
                            static_cast<uint32_t>(std::lround(max_neg_deg * 100.0)));
}

// 0xB3 通讯中断保护时间（ms，0=关闭，写 ROM）：超时切断输出并锁死抱闸
bool Motor::set_com_protect_time(uint32_t time_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_com_protect_time(id_, time_ms),
                 static_cast<uint8_t>(RhCmd::SetComProtectTime), reply)) {
        return false;
    }
    // 0xB3 回复为原帧回显：校验命令字节即可（数据字段无需回读）
    return reply.data[0] == static_cast<uint8_t>(RhCmd::SetComProtectTime);
}

// 0x9A 读状态1：温度/MOS 温度/抱闸/电压/错误标志
bool Motor::read_status(MotorStatus& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::ReadStatus),
                 static_cast<uint8_t>(RhCmd::ReadStatus), reply)) {
        return false;
    }
    return decode_status(reply, out);
}

// 0x9C 读状态2：温度/转矩电流/转速/角度
bool Motor::read_run_status(MotorRunStatus& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::ReadStatus2),
                 static_cast<uint8_t>(RhCmd::ReadStatus2), reply)) {
        return false;
    }
    return decode_run_status(reply, out);
}

// 0x92 读多圈绝对角度（°，0.01°/LSB）
bool Motor::read_angle(double& angle_deg) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::ReadAngle),
                 static_cast<uint8_t>(RhCmd::ReadAngle), reply)) {
        return false;
    }
    return decode_angle(reply, angle_deg);
}

// 0x9D 读状态3：温度 + 三相相电流 iA/iB/iC
bool Motor::read_status3(MotorStatus3& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::ReadStatus3),
                 static_cast<uint8_t>(RhCmd::ReadStatus3), reply)) {
        return false;
    }
    return decode_status3(reply, out);
}

// 0x90 读单圈编码器 encoder/raw/offset（脉冲，直驱用）
bool Motor::read_single_encoder(SingleEncoder& enc) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::ReadSingleEncoder),
                 static_cast<uint8_t>(RhCmd::ReadSingleEncoder), reply)) {
        return false;
    }
    return decode_single_encoder(reply, enc);
}

// 0xB1 读系统运行时间（自复位起，ms）
bool Motor::read_run_time(uint32_t& time_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::ReadRunTime),
                 static_cast<uint8_t>(RhCmd::ReadRunTime), reply)) {
        return false;
    }
    return decode_run_time(reply, time_ms);
}

// 0xB2 读软件版本日期（yyyymmdd）
bool Motor::read_version_date(uint32_t& date) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::ReadVersionDate),
                 static_cast<uint8_t>(RhCmd::ReadVersionDate), reply)) {
        return false;
    }
    return decode_version_date(reply, date);
}

// 0xB5 读电机型号（7 个 ASCII 字符）
bool Motor::read_motor_model(char model[8]) {
    std::lock_guard<std::mutex> lock(mutex_);
    CanFrame reply;
    if (!request(encode_command(id_, RhCmd::ReadMotorModel),
                 static_cast<uint8_t>(RhCmd::ReadMotorModel), reply)) {
        return false;
    }
    return decode_motor_model(reply, model);
}

// 0xA6 单圈位置闭环（直驱用）：direction 0=顺时针/1=逆时针，angle_deg 目标单圈角度（0°~359.99°）
bool Motor::set_single_angle_position(uint8_t direction, uint16_t max_speed_dps,
                                      double angle_deg, MotorRunStatus* out) {
    return control(encode_single_angle_position(id_, direction, max_speed_dps, angle_deg),
                   static_cast<uint8_t>(RhCmd::SingleAnglePos), out);
}

// 开始录制：打开文件写表头、记录起点，再启动录制线程
bool Motor::start_record(const std::string& path, std::chrono::milliseconds interval) {
    if (record_running_) {
        return false;  // 已在录制，调用方需先 stop_record
    }
    record_file_.open(path, std::ios::out | std::ios::trunc);
    if (!record_file_.is_open()) {
        return false;  // 路径不可写 / 目录不存在
    }
    // 统一 fixed + 3 位小数，CSV 各列格式一致
    record_file_ << std::fixed << std::setprecision(3);
    record_file_ << "motor_id,t_s,voltage_v,speed_dps,angle_deg,iq_a\n";

    record_start_ = std::chrono::steady_clock::now();
    record_interval_ = interval > std::chrono::milliseconds(0)
                           ? interval
                           : std::chrono::milliseconds(100);
    record_stop_ = false;
    record_running_ = true;
    record_thread_ = std::thread(&Motor::record_loop, this);
    return true;
}

// 停止录制：置停止标志并 join 录制线程（最坏阻塞一个 reply_timeout），然后冲刷关闭文件
void Motor::stop_record() {
    if (!record_running_) {
        return;  // 幂等：未录制时 no-op
    }
    record_stop_ = true;
    if (record_thread_.joinable()) {
        record_thread_.join();
    }
    record_file_.flush();
    record_file_.close();
    record_running_ = false;
}

// 录制线程体：每 interval 读 0x9A + 0x9C，两者都成功才写一行（不补失败空拍，
// 时间列按实际采样时刻计，如实反映通讯中断导致的间隔）；t_s 为距开始录制的秒数
void Motor::record_loop() {
    while (!record_stop_) {
        const auto deadline = std::chrono::steady_clock::now() + record_interval_;
        MotorStatus st;
        MotorRunStatus rs;
        if (read_status(st) && read_run_status(rs)) {
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - record_start_)
                                 .count();
            record_file_ << static_cast<int>(id_) << ',' << t << ',' << st.voltage_v
                         << ',' << rs.speed_dps << ',' << rs.angle_deg << ',' << rs.iq_a
                         << '\n';
            record_file_.flush();  // 每拍冲刷，异常退出时最多丢一拍的缓冲数据
        }
        // 睡到下一拍；读耗时超出一拍（电机无响应）时直接续下一循环，不补积压
        std::this_thread::sleep_until(deadline);
    }
}

}  // namespace motor_can
