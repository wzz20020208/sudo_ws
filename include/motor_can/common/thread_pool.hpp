// include/motor_can/common/thread_pool.hpp
// 最小线程池：固定 N 个工作线程 + 线程安全任务队列。
//
// 用途：多机控制（MultiMotorController）批量提交对不同电机的控制指令，
// 由池内工作线程异步发布到对应 Motor —— 不同电机并行，同一电机靠 Motor
// 内部锁串行。
//
// 生命周期：析构置停止标志并 notify_all，工作线程处理完队列剩余任务后退出，
// 主线程 join 全部工作线程，不会丢任务、不会悬挂。
#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

namespace motor_can {

class ThreadPool {
public:
    /// 启动 threads 个工作线程。threads=0 时退化为「提交即返回 false」。
    explicit ThreadPool(size_t threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    /// 提交一个任务到队列，由工作线程异步执行。
    /// @return true=已入队；false=线程池已停止（析构中或析构后）。
    bool submit(std::function<void()> task);

private:
    void worker_loop();

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stopping_ = false;
};

}  // namespace motor_can
