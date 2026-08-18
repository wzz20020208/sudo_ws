// include/motor_can/motor/multi_motor.hpp
// 多机控制（Device 层）：多机共享一个 CanComm，批量接收不同电机的控制指令，
// 由线程池异步发布到对应 Motor。
//
//  - 一个 CanComm（单 socket）服务于总线上所有电机：接收线程按 ID 分队列，
//    各 Motor 的 request() 只收自己的回复，杂帧由命令字节过滤丢弃。
//  - 批量提交接口 submit_*：入队即返回，线程池工作线程异步发布到对应 Motor。
//  - 不同电机并行执行；同一电机由 Motor 内部锁串行（保证请求/回复过滤不串线）。
//
// 生命周期：构造打开共享 CanComm + 起线程池；电机用 add_motor() 逐台添加
// （home_on_init=true 时添加即物理归0，真实运动）。析构先 join 线程池（任务
// 执行完）再拆电机、关连接，不会出现任务悬空访问已销毁对象。
// add_motor() 只应在启动阶段调用（会改 motors_ 容器，与 submit_* 并发不安全）。
#pragma once

#include "motor_can/common/thread_pool.hpp"
#include "motor_can/motor/motor.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

namespace motor_can {

class MultiMotorController {
public:
    /// 打开共享 CanComm 并接管其生命周期 + 起线程池；电机用 add_motor() 逐台添加。
    /// @param pool_threads 线程池工作线程数（0 则 submit 全部失败）
    MultiMotorController(const CanConfig& can_config, size_t pool_threads);
    ~MultiMotorController() = default;

    MultiMotorController(const MultiMotorController&) = delete;
    MultiMotorController& operator=(const MultiMotorController&) = delete;

    bool is_open() const noexcept { return comm_.is_open(); }

    // ---- 机群管理（启动阶段调用；会改 motors_ 容器，勿与 submit_* 并发）----

    /// 添加一台电机并建立其 Motor（默认 Config：home_on_init=true，添加即物理归0）。
    /// @return false=总线未打开 或 该 ID 已在管理集合中。
    bool add_motor(uint8_t id);

    /// 添加一台电机，使用独立 Config（可逐台差异化限速/限扭/是否归0）。
    bool add_motor(uint8_t id, const Motor::Config& config);

    // ---- 批量提交（入队即返回；线程池异步发布），true=已入队 ----

    bool submit_current(uint8_t id, double current_a);   ///< 0xA1 转矩环
    bool submit_speed(uint8_t id, double speed_dps, uint8_t max_torque_pct);  ///< 0xA2
    bool submit_position(uint8_t id, double angle_deg, uint16_t max_speed_dps);  ///< 0xA4
    bool submit_stop(uint8_t id);                        ///< 0x81 停止
    bool submit_brake(uint8_t id, bool release);         ///< true=0x77 开闸, false=0x78 锁闸
    bool submit_home(uint8_t id);                        ///< 物理归0

    // ---- 0x280 广播（所有电机同时响应，各自在 0x240+ID 回复；前置：先失能 0x20/0x02
    //      CANID 滤波器）。发送后不等回复，返回发送是否成功。----

    bool broadcast_stop();       ///< 0x280 + 0x81 广播停止所有电机
    bool broadcast_off();        ///< 0x280 + 0x80 广播关闭所有电机输出（清除运行状态）
    bool broadcast_position(double angle_deg, uint16_t max_speed_dps);  ///< 0x280 + 0xA4

    // ---- 同步读取（监控/测试用，不走线程池）----

    bool read_status(uint8_t id, MotorStatus& out);      ///< 0x9A
    bool read_angle(uint8_t id, double& angle_deg);      ///< 0x92

    /// 管理中的电机 ID 列表（供上层遍历 / 初始化清单）。
    std::vector<uint8_t> ids() const;

private:
    Motor* find(uint8_t id);  ///< id 不在管理集合时返回 nullptr

    CanComm comm_;                                        // 共享连接（先声明，最后销毁）
    std::map<uint8_t, std::unique_ptr<Motor>> motors_;    // 每机一个 Motor，构造即归0
    ThreadPool pool_;                                     // 后声明，先销毁（先 join 再拆电机）
};

}  // namespace motor_can
