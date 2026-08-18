# CAN 传输层设计（重构版）

## 目标

提供底层 CAN 收发（当前实现 SocketCAN），形态为**「单一具体类 + 接收线程」**
（`CanComm`），为多电机控制做准备。

- 底层职责只到「收发一帧」，不关心帧的协议含义（协议含义在里程碑 2 协议层）。
- 多电机通过「按 ID 分类的收帧队列」支持：每台电机的应答各归各的队列。

> 重构前的里程碑 1 实现（`CanDriver` 抽象 + `SocketCanDriver` + 工厂）已存档到
> `third_party/reference/`，本文件为重构后的设计记录。

## 目录结构

按「电机控制视角」分文件夹，`include/` 与 `src/` 同名一一对应：

```
include/motor_can/
├── can_comm/         # CAN 底层通讯：本文件所设计的 CanComm
│   ├── can_types.hpp # 公共类型：CanFrame / CanConfig / CanError
│   └── can_comm.hpp  # CanComm 类声明
├── common/
│   └── log.hpp       # 轻量日志宏 MC_LOG_*（header-only，无第三方依赖）
├── protocol/         # 电机协议（里程碑 2，RH 指令编解码）
└── motor/            # 电机控制（里程碑 3，闭环控制）
src/
├── can_comm/         # 底层通讯实现（can_comm.cpp）
└── tests/
    └── can_comm_test.cpp # open() 自测程序（cmake 目标 can_comm_test）
```

> 以电机控制模块划分目录，而非抽象分层：将来接入 USB-CAN 只在 `can_comm/`
> 内新增实现文件，不需要新建目录。

## 公共类型（`include/motor_can/can_comm/can_types.hpp`，已定稿）

### CanFrame —— 一帧 CAN 报文

```cpp
struct CanFrame {
    uint32_t id;            // 帧 ID（不含 EFF/RTR 标志位）
    bool     is_extended;   // true=扩展帧(29bit)  false=标准帧(11bit)
    uint8_t  dlc;           // 数据长度 0..8
    uint8_t  data[8];       // 数据字节
};
```

不直接暴露 Linux `struct can_frame`：SocketCAN / USB-CAN 各实现的收发结构都能
翻译成本类型，上层（协议层、设备层）不感知底层硬件差异。

### CanConfig —— 接口配置

```cpp
struct CanConfig {
    std::string ifname = "can0";    // SocketCAN 接口名（如 can0）
    uint32_t    bitrate = 1000000;  // 波特率（bps）；RH 电机要求 1Mbps
    bool        loopback = false;   // true=接收自己发送的帧（自测用）；真实电机保持 false
};
```

### CanError —— 错误码

```cpp
enum class CanError {
    None,           // 无错误
    SendFailed,     // 发送失败
    ReceiveTimeout, // 接收超时
    BusOff,         // 总线关闭（严重错误）
    Closed,         // 接口已关闭
    Unknown,        // 未知错误
};
```

## 底层通讯类 CanComm（重构中，逐步实现）

### 形态与线程模型

- 单一具体类（暂不做抽象接口），当前只实现 SocketCAN。
- **接收**：1 个接收线程阻塞读 socket，收到帧按 ID 塞进对应队列。
- **发送**：需要发的线程直接 `write()`，由互斥锁保护（多电机多线程安全）。
- **队列**：`std::map<uint32_t, std::deque<CanFrame>>`，每 ID 队列设深度上限
  （超限丢最旧帧），把内存占用硬性封顶；每个电机的应答从自己的队列取，
  避免多电机应答串线。
- **停止**：`close()` 先置停止标志，再关闭 fd 使阻塞的 `read()` 返回，`join()` 线程。

### 接口

```cpp
class CanComm {
public:
    ~CanComm();                                             // RAII：析构自动 close()

    bool open(const CanConfig& config);                     // 打开 socket + 启动接收线程
    void close();                                           // 停接收线程 + 关 socket
    bool is_open() const noexcept;

    bool send(const CanFrame& frame);                       // 发送一帧（线程安全）
    bool receive(CanFrame& frame, std::chrono::milliseconds timeout_ms);          // 取任一帧
    bool receive_by_id(uint32_t id, CanFrame& frame,
                       std::chrono::milliseconds timeout_ms);                     // 按 ID 等应答
};
```

> `receive_by_id` 为多电机控制的核心语义：协议层/设备层发完指令后，
> 只需等待 `0x240+自己的ID` 的应答，不会取到其他电机的帧。

### 波特率约定

`CanConfig::bitrate` **仅作记录**（期望值，供诊断/日志），不实际设置波特率：
设置波特率需要 root（`sudo ip link set can0 type can bitrate …`），是调用方职责，
须在 `open()` 之前完成。`open()` 只做打开与绑定，并检查接口 `IFF_UP`，
未开启则失败返回。

### 与后续里程碑的关系

- 协议层（里程碑 2）：只使用 `send` + `receive_by_id`，CanComm 对其透明。
- 设备层（里程碑 3）：每台电机一个控制线程，各自 `send` + `receive_by_id`。
- 将来支持 USB-CAN：新增一个实现，接口不变（抽象接口届时视需要引入）。

### 实现进度

| 步骤 | 内容 | 状态 |
| :-- | :-- | :-- |
| Step 1 | 公共类型 + 类声明（头文件） | ✅ 完成 |
| Step 2 | `open()`（打开 socket + 启动接收线程） | ✅ 完成（已实测通过） |
| — 配套 | `~CanComm()` / `close()` / `is_open()` / 最小 `receive_loop()` | ✅ 完成（已随 open 实测） |
| Step 4 | `send()` | ✅ 完成（实机通过） |
| Step 5 | `receive()` | ✅ 完成（实机通过） |
| Step 6 | `receive_by_id()` | ✅ 完成（实机通过：motor_spin_test 全指令收到回复） |
| — 收尾 | 完善 `receive_loop()`（含 EINTR/深度上限复核）、补文档 | ⏳ 待办 |

> 遵守「不要一步到位」：每个函数实现前与用户确认方案，实现后确认结果。

### 关键实现要点（已写代码）

- **fd 并发安全**：接收线程在互斥锁内把 `fd_` 拷贝到局部变量，`read()` 阻塞期间不持锁；
  `close()` 也在锁内将 `fd_` 置 -1。`running_` 为 `std::atomic<bool>`，`is_open()` 由它返回。
- **停止顺序**：`close()` 先 `running_=false` → 关 fd（阻塞的 `read()` 返回失败）→ `join()`。
- **入队**：`map<id, deque>`，每 ID 深度上限 `kMaxQueueDepth=64`，超限丢最旧；
  入队后 `cv_.notify_all()`（供后续 `receive`/`receive_by_id` 用）。
- **发送**：`send()` 锁内快照 `fd_`、锁外 `write()`——互斥锁只保护 fd 本身；CAN_RAW
  的 `write()` 一帧原子完成，多线程并发写由内核串行化，帧不会交错，无需为写加锁。
  `EINTR` 重试。发送超时（`SO_SNDTIMEO`）预留占位，当前未启用。
- **接收**：`receive()` 等待「任一队列有帧或已关闭」，取第一张非空队列队首；`receive_by_id(id)`
  等待「指定 ID 队列有帧或已关闭」，只取该 ID 的队首。均用 `cv_` + 谓词等待，超时或关闭返回
  `false`；队列取空即从 map 删除，保持无空队列。`close()` 在清队列后 `cv_.notify_all()`，
  唤醒等待中的 receive 使其发现已关闭退出。
- **测试程序** `src/tests/can_comm_test.cpp`：三个用例 —— ① 正常 open/close（默认 can0）
  ② open(can99) 失败路径 ③ 接口未 UP 时 open 失败。构建 `cd build && cmake .. && make -j`，
  运行 `./build/can_comm_test [--ifname <if>]`（接口需先 `sudo ip link set can0 type can bitrate 1000000 && up`）。
