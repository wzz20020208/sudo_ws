
// src/can_comm/can_comm.cpp
// CanComm 实现：SocketCAN 底层通讯。
// 已实现：open / close / is_open / send / receive / receive_by_id / 接收线程 / 析构。
//
// 波特率说明：bitrate 仅作记录（期望值），实际配置需调用方在 open 之前用
// `sudo ip link set <ifname> type can bitrate <值>` 完成 —— 程序内改波特率需要
// root 权限，不属于本类职责。

#include "motor_can/can_comm/can_comm.hpp"

#include "motor_can/common/log.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace motor_can {

namespace {
// 每个 ID 的收帧队列深度上限：超限丢最旧帧，把内存占用封顶
// （32 电机 × 256 帧 × 约 16 字节 ≈ 128KB）。深度取大值是为了抗共享总线上的
// 外部帧挤掉本进程等待的回复（如跟随 demo 读电机1 角度时被外部控制帧淹没）。
constexpr size_t kMaxQueueDepth = 256;
// 接收线程 read() 单次阻塞上限：让 close() 置停止标志后最迟 100ms 内被唤醒退出，
// 不依赖「跨线程 close 唤醒阻塞读」（Linux 不保证，实测会死锁 join()）。
constexpr timeval kRxTimeout{/*tv_sec*/ 0, /*tv_usec*/ 100 * 1000};
}  // namespace

CanComm::~CanComm() {
    close();
}

bool CanComm::open(const CanConfig& config) {
    if (is_open()) {
        MC_LOG_ERROR("CanComm 已打开，不能重复 open()");
        return false;
    }

    // 1. 建 CAN 原始套接字
    const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        MC_LOG_ERROR("socket(PF_CAN) 失败: %s", std::strerror(errno));
        return false;
    }

    // 2. 接口名 -> 接口索引（名字超长先拒掉，避免截断）
    if (config.ifname.size() >= IFNAMSIZ) {
        MC_LOG_ERROR("接口名 %s 过长", config.ifname.c_str());
        ::close(fd);
        return false;
    }
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, config.ifname.c_str(), IFNAMSIZ);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        MC_LOG_ERROR("找不到接口 %s: %s", config.ifname.c_str(), std::strerror(errno));
        ::close(fd);
        return false;
    }

    // 3. 回环模式：true=也接收自己发送的帧（loopback 自测用）；真实电机保持 false
    const int loopback = config.loopback ? 1 : 0;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
        MC_LOG_WARN("setsockopt(CAN_RAW_LOOPBACK) 失败（忽略，不影响收发）: %s",
                    std::strerror(errno));
    }

    // 3.5 接收超时：让接收线程的 read() 最多阻塞 kRxTimeout，否则 close() 后
    //     join() 可能因「跨线程 close 不唤醒阻塞读」而死锁（见 receive_loop）。
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &kRxTimeout, sizeof(kRxTimeout)) < 0) {
        MC_LOG_WARN("setsockopt(SO_RCVTIMEO) 失败（析构可能死锁）: %s", std::strerror(errno));
    }

    // 4. 绑定到该接口
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<const struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        MC_LOG_ERROR("bind(%s) 失败: %s", config.ifname.c_str(), std::strerror(errno));
        ::close(fd);
        return false;
    }

    // 5. 检查接口是否已 UP（波特率等配置由调用方提前用 sudo ip link set 完成）
    if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0) {
        MC_LOG_ERROR("ioctl(SIOCGIFFLAGS) 失败: %s", std::strerror(errno));
        ::close(fd);
        return false;
    }
    if ((ifr.ifr_flags & IFF_UP) == 0) {
        MC_LOG_ERROR("接口 %s 未开启，请先 `sudo ip link set %s up`",
                     config.ifname.c_str(), config.ifname.c_str());
        ::close(fd);
        return false;
    }

    // 6. 记录 fd，启动接收线程
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fd_ = fd;
    }
    running_.store(true);
    try {
        rx_thread_ = std::thread(&CanComm::receive_loop, this);
    } catch (const std::system_error& e) {
        running_.store(false);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            fd_ = -1;
        }
        ::close(fd);
        MC_LOG_ERROR("创建接收线程失败: %s", e.what());
        return false;
    }

    MC_LOG_INFO("CanComm open 成功: %s @ %u bps%s", config.ifname.c_str(), config.bitrate,
                config.loopback ? "（loopback）" : "");
    return true;
}

void CanComm::close() {
    int fd;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fd_ < 0) {
            return;  // 未打开或已关闭；重复 close() 安全
        }
        running_.store(false);  // 先置停止标志
        fd = fd_;
        fd_ = -1;
    }
    // 关闭 fd 使阻塞中的 read() 返回失败，接收线程随即退出
    ::close(fd);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_queues_.clear();  // 清掉残留帧，下次 open 从干净状态开始
    }
    cv_.notify_all();  // 唤醒等待 receive 的线程，使其发现接口已关闭后退出
    MC_LOG_INFO("CanComm 已关闭");
}

bool CanComm::is_open() const noexcept {
    return running_.load();
}

bool CanComm::send(const CanFrame& frame) {
    // 1. 锁内快照 fd_：互斥锁只保护 fd 本身，写帧在锁外进行
    int fd;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fd = fd_;
    }
    if (fd < 0) {
        MC_LOG_ERROR("send() 失败：接口未打开");
        return false;
    }

    // 2. CanFrame -> 内核 can_frame
    struct can_frame f {};
    f.can_id = frame.id;
    if (frame.is_extended) {
        f.can_id |= CAN_EFF_FLAG;  // 扩展帧标志；标准帧只含 11bit ID
    }
    f.can_dlc = frame.dlc;
    std::copy_n(frame.data, 8, f.data);

    // 3. 阻塞写一帧：CAN_RAW 的 write() 一次发送完整一帧，多线程并发写由内核
    //    串行化（帧不会交错），因此无需互斥锁保护写本身。
    //    [预留占位] 若需在 TX 队列满时避免无限阻塞，可在此设置 SO_SNDTIMEO；
    //    当前电机控制场景 TX 队列几乎不会满，暂不启用。
    //    计时 write 并只在失败时打印：send 失败（如 ENOBUFS）时用耗时判断是排队超时
    //    还是瞬时满（成功帧不打，避免 100Hz 刷屏）。
    const auto t0 = std::chrono::steady_clock::now();
    ssize_t written;
    do {
        written = ::write(fd, &f, sizeof(f));
    } while (written < 0 && errno == EINTR);  // 被信号打断则重试
    const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - t0)
                                .count();

    if (written != static_cast<ssize_t>(sizeof(f))) {
        MC_LOG_ERROR("send() 失败: %s（耗时 %lld us，id=0x%X）", std::strerror(errno),
                     static_cast<long long>(elapsed_us), static_cast<unsigned>(frame.id));
        return false;
    }
    return true;
}

bool CanComm::receive(CanFrame& frame, std::chrono::milliseconds timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待条件：任一队列有帧，或接口已关闭（close() 时被唤醒退出，不死等）
    const auto pred = [this] {
        return !rx_queues_.empty() || !running_.load();
    };
    if (!cv_.wait_for(lock, timeout_ms, pred)) {
        return false;  // 超时未取到
    }
    if (!running_.load()) {
        return false;  // 接口已关闭
    }

    // 取第一张非空队列的队首帧（map 按键有序，即 ID 最小的队列）
    auto it = rx_queues_.begin();
    frame = it->second.front();
    it->second.pop_front();
    if (it->second.empty()) {
        rx_queues_.erase(it);  // 取空即删，保持队列集合无空项
    }
    return true;
}

bool CanComm::receive_by_id(uint32_t id, CanFrame& frame,
                            std::chrono::milliseconds timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);

    // 等待条件：指定 ID 的队列有帧，或接口已关闭
    const auto pred = [this, id] {
        auto it = rx_queues_.find(id);
        return (it != rx_queues_.end() && !it->second.empty()) || !running_.load();
    };
    if (!cv_.wait_for(lock, timeout_ms, pred)) {
        return false;  // 超时未取到
    }
    if (!running_.load()) {
        return false;  // 接口已关闭
    }

    // 取指定 ID 队列的队首帧；队列取空即删
    auto it = rx_queues_.find(id);
    frame = it->second.front();
    it->second.pop_front();
    if (it->second.empty()) {
        rx_queues_.erase(it);
    }
    return true;
}

void CanComm::receive_loop() {
    while (running_.load()) {
        int fd;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            fd = fd_;
        }
        if (fd < 0) {
            break;
        }

        struct can_frame frame {};
        const ssize_t n = ::read(fd, &frame, sizeof(frame));
        if (n < 0 && errno == EINTR) {
            continue;  // 被信号打断，重试
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;  // SO_RCVTIMEO 超时无帧：回头检查 running_，close() 后即退出
        }
        if (n != static_cast<ssize_t>(sizeof(frame))) {
            break;  // fd 被 close() 关闭（EBADF）→ 结束线程
        }

        // 内核帧 -> CanFrame，按 ID 入队
        CanFrame f;
        f.id = frame.can_id & CAN_EFF_MASK;
        f.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
        f.dlc = frame.can_dlc;
        std::copy_n(frame.data, 8, f.data);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::deque<CanFrame>& q = rx_queues_[f.id];
            if (q.size() >= kMaxQueueDepth) {
                q.pop_front();  // 超限丢最旧
            }
            q.push_back(f);
        }
        cv_.notify_all();
    }
}

}  // namespace motor_can
