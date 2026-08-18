// src/common/thread_pool.cpp
// ThreadPool 实现：N 工作线程 + 任务队列，mutex/condvar 同步。

#include "motor_can/common/thread_pool.hpp"

namespace motor_can {

ThreadPool::ThreadPool(size_t threads) {
    if (threads == 0) {
        stopping_ = true;  // 0 线程：立即进入停止态，submit 直接返回 false
        return;
    }
    workers_.reserve(threads);
    for (size_t i = 0; i < threads; ++i) {
        workers_.emplace_back(&ThreadPool::worker_loop, this);
    }
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    for (std::thread& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

bool ThreadPool::submit(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) {
            return false;
        }
        tasks_.push(std::move(task));
    }
    cv_.notify_one();
    return true;
}

void ThreadPool::worker_loop() {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            // 停止且队列清空才退；停止但还有任务 → 先执行完剩余任务再退
            cv_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
            if (stopping_ && tasks_.empty()) {
                return;
            }
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        task();
    }
}

}  // namespace motor_can
