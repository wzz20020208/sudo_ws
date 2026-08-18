// include/motor_can/can_comm/can_comm.hpp
// CAN 底层通讯类：封装 SocketCAN，含接收线程与按 ID 分类的收帧队列。
//
// 线程模型：
//  - 接收：1 个接收线程阻塞读 socket，收到帧按 ID 塞进对应队列。
//  - 发送：多线程可直接调用 send()，由互斥锁保护。
//  - 队列：map<id, deque>，每 ID 深度上限，超限丢最旧，内存封顶。
//
// 多电机语义：每台电机的应答落在各自的 ID 队列，receive_by_id() 只取
// 自己那台电机的应答，不会串线。
#pragma once

#include "motor_can/can_comm/can_types.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <thread>

namespace motor_can {

class CanComm {
public:
    // ---- 生命周期 ----

    /// RAII：析构时自动 close()，回收接收线程，避免线程悬挂。
    ~CanComm();

    /// 打开 SocketCAN 接口并启动接收线程。
    /// @param config 接口配置（ifname / bitrate / loopback）
    /// @return true=成功；false=失败（接口不存在、未配置等）
    bool open(const CanConfig& config);

    /// 停止接收线程并关闭接口；可安全重复调用。
    void close();

    /// 是否处于打开状态。
    bool is_open() const noexcept;

    // ---- 收发（线程安全）----

    /// 发送一帧。
    /// @return true=成功；false=失败（接口未开、总线错误等）
    bool send(const CanFrame& frame);

    /// 从收帧队列取任意一帧。
    /// @param[out] frame 取出的帧
    /// @param timeout_ms 等待超时
    /// @return true=取到一帧；false=超时或接口已关闭
    bool receive(CanFrame& frame, std::chrono::milliseconds timeout_ms);

    /// 按 ID 等待一帧应答（多电机场景的核心语义）。
    /// @param id 期望的帧 ID（如 0x240 + 电机ID）
    /// @param[out] frame 取出的帧
    /// @param timeout_ms 等待超时
    /// @return true=取到对应 ID 的帧；false=超时或接口已关闭
    bool receive_by_id(uint32_t id, CanFrame& frame,
                       std::chrono::milliseconds timeout_ms);

private:
    void receive_loop();   ///< 接收线程入口：阻塞读 socket -> 按 ID 入队

    int  fd_ = -1;         ///< SocketCAN 文件描述符
    std::thread rx_thread_;  ///< 接收线程
    std::mutex mutex_;       ///< 保护收帧队列与关闭状态
    std::condition_variable cv_;  ///< 队列非空通知
    std::map<uint32_t, std::deque<CanFrame>> rx_queues_;  ///< 按 ID 分类的收帧队列
    std::atomic<bool> running_{false};  ///< 接收线程运行标志
};

}  // namespace motor_can
