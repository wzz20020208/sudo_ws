// tests/thread_pool_test.cpp
// ThreadPool 自测（纯逻辑，不碰硬件）：
//   1. 100 个任务全部执行（析构 join 后计数正确）
//   2. 多线程并发执行（活跃数 > 1 证明并行）
//   3. 0 线程池 submit 返回 false
// 用法: ./thread_pool_test

#include "motor_can/common/log.hpp"
#include "motor_can/common/thread_pool.hpp"

#include <atomic>
#include <chrono>
#include <thread>

namespace {

int g_failures = 0;

void expect(bool cond, const char* what) {
    if (!cond) {
        ++g_failures;
        MC_LOG_ERROR("FAIL: %s", what);
    } else {
        MC_LOG_INFO("PASS: %s", what);
    }
}

}  // namespace

int main() {
    // 1. N 任务全部执行：作用域内提交，析构 join 后计数必须 == N
    {
        std::atomic<int> counter{0};
        bool all_queued = true;
        {
            motor_can::ThreadPool pool(4);
            constexpr int kTasks = 100;
            for (int i = 0; i < kTasks; ++i) {
                all_queued = pool.submit([&counter] { ++counter; }) && all_queued;
            }
        }  // 析构 join，任务必须全部跑完
        expect(all_queued, "100 个任务全部入队返回 true");
        expect(counter == 100, "析构 join 后 100 个任务全部执行");
    }

    // 2. 并发执行：4 线程 + 8 个各睡 50ms 的任务，活跃数应 > 1（有并行）
    {
        std::atomic<int> active{0};
        std::atomic<int> max_active{0};
        {
            motor_can::ThreadPool pool(4);
            for (int i = 0; i < 8; ++i) {
                pool.submit([&active, &max_active] {
                    const int now = ++active;
                    int cur = max_active.load();
                    while (now > cur && !max_active.compare_exchange_weak(cur, now)) {
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    --active;
                });
            }
        }
        expect(max_active > 1, "4 线程执行 8 个任务时并发活跃数 > 1");
        MC_LOG_INFO("INFO 最大并发活跃数 = %d", max_active.load());
    }

    // 3. 0 线程池：submit 直接返回 false
    {
        motor_can::ThreadPool zero(0);
        expect(!zero.submit([] {}), "0 线程池 submit 返回 false");
    }

    if (g_failures > 0) {
        MC_LOG_ERROR("%d 个用例失败", g_failures);
        return 1;
    }
    MC_LOG_INFO("全部用例通过");
    return 0;
}
