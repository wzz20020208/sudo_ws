# Device 层设计（里程碑 3）

## 目标

组合传输层（CanComm）+ 协议层（rh_protocol），暴露给上层两台控制器：

- **单机控制 `Motor`**：初始化（构造完成 CAN 连接 + 物理归0）+ 三环控制接口，
  并内置**请求/回复过滤**（解决里程碑 2 遗留的总线共享串扰）。
- **多机控制 `MultiMotorController`**：多机共享一个 CanComm，批量接收不同电机的
  控制指令，由线程池异步发布到对应 `Motor`。

## 目录与文件

```
include/motor_can/common/thread_pool.hpp    # 最小线程池
include/motor_can/motor/motor.hpp           # 单机控制 Motor
include/motor_can/motor/multi_motor.hpp     # 多机控制 MultiMotorController
src/common/thread_pool.cpp
src/motor/motor.cpp
src/motor/multi_motor.cpp
src/tests/thread_pool_test.cpp              # 纯逻辑（不碰硬件）
src/tests/motor_device_test.cpp             # 实机（Phase A: Motor 单机 / Phase B: 多机冒烟）
```

CMake 目标：`motor_can_common`（PUBLIC include 目录）、`motor_can_motor`
（PUBLIC 依赖 can_comm / protocol / common）。

## 接口

### ThreadPool

```cpp
class ThreadPool {
public:
    explicit ThreadPool(size_t threads);    // 0 线程 = 立即停止态，submit 恒 false
    ~ThreadPool();                          // 置停 + notify + join（处理完剩余任务）
    bool submit(std::function<void()> task); // true=已入队
};
```

### Motor（单机控制）

```cpp
class Motor {
public:
    struct Config {
        uint16_t max_speed_dps = 30;       // 位置环默认限速 °/s
        uint8_t  max_torque_pct = 30;      // 速度环默认限扭 %
        bool     home_on_init = true;      // 构造时物理归0（真实运动）
        std::chrono::milliseconds reply_timeout{500};
    };
    Motor(CanComm& comm, uint8_t id);                 // 默认配置
    Motor(CanComm& comm, uint8_t id, const Config&);  // comm 须已 open

    // 三环控制：成功返回 true，out 回读控制命令回复的运行状态
    bool set_current(double current_a, MotorRunStatus* out = nullptr);         // 0xA1
    bool set_speed(double speed_dps, MotorRunStatus* out = nullptr);           // 0xA2 默认限扭
    bool set_speed(double speed_dps, uint8_t max_torque_pct, MotorRunStatus* out = nullptr);
    bool set_position(double angle_deg, MotorRunStatus* out = nullptr);        // 0xA4 默认限速
    bool set_position(double angle_deg, uint16_t max_speed_dps, MotorRunStatus* out = nullptr);

    bool home();                    // 物理归0：0x77 开闸 → 0xA4 到 0°
    bool brake_release();  bool brake_lock();  bool stop();  bool off();

    // 参数配置 / 零点（走请求/回复过滤）
    bool read_pid(PidIndex, float&);  bool write_pid_ram(PidIndex, double);
    bool write_pid_rom(PidIndex, double);  bool set_zero_point(int32_t& new_offset);

    bool read_status(MotorStatus&);  bool read_run_status(MotorRunStatus&);  bool read_angle(double&);
};
```

### MultiMotorController（多机控制）

```cpp
class MultiMotorController {
public:
    MultiMotorController(const CanConfig& can_config,
                         size_t pool_threads);  // 开共享 CanComm + 起线程池
    bool is_open() const;

    // 机群管理：启动阶段逐台添加（会改 motors_ 容器，勿与 submit_* 并发）
    bool add_motor(uint8_t id);                              // 默认 Config（添加即归0）
    bool add_motor(uint8_t id, const Motor::Config& config); // 每台独立 Config

    bool submit_current(uint8_t id, double A);                          // 0xA1
    bool submit_speed(uint8_t id, double dps, uint8_t max_torque_pct);  // 0xA2
    bool submit_position(uint8_t id, double deg, uint16_t max_speed_dps); // 0xA4
    bool submit_stop(uint8_t id);  bool submit_brake(uint8_t id, bool release);
    bool submit_home(uint8_t id);
    bool read_status(uint8_t id, MotorStatus&);  bool read_angle(uint8_t id, double&);
    std::vector<uint8_t> ids() const;
};
```

> 两台电机用法：`ctrl.add_motor(1); ctrl.add_motor(2);` —— 各建 Motor（添加即归0），
> 共享一个 CanComm，submit_* 按 id 并行发布。

## 关键机制：请求/回复过滤

CAN 总线广播：总线上所有 socket 都能收到同一台电机的回复（GUI 轮询 / 其他电机 /
其他进程的帧都会进本进程的接收队列）。`Motor::request()` 解决：

```
发一帧 → 循环 receive_by_id(0x240+id, ...) 直到 reply.data[0] == 期望命令字节：
  - 匹配 → 收下并解码
  - 不匹配 → 杂帧（其他进程/电机/查询的回复），直接丢弃，继续收
总超时用 steady_clock 从发出时刻递减剩余时间，杂帧不会把超时无限拖长。
```

- 控制命令（0xA1/A2/A4）回复命令字节 = 命令本身，与 0x9C 布局一致 → `decode_run_status`。
- 单字节命令（0x77/0x78/0x80/0x81）回复为原帧回显，命令字节 = 命令本身。
- 0x76 系统复位无回复（不用于 Motor 接口）。

## 线程模型与生命周期

- **Motor 内部 `std::mutex`**：同一台电机的收发串行化（保证过滤不串线）；
  不同电机实例各自持锁 → 互不阻塞。
- **线程池发布**：不同电机的 submit_* 并行执行；同一电机因 Motor 锁天然串行。
- **生命周期**：MultiMotorController 成员顺序 `comm_ → motors_ → pool_`（后声明先销毁）：
  析构先 join 线程池（任务执行完，含 Motor 原始指针捕获的引用仍存活），再拆电机，最后关连接。
- 构造 `home_on_init=true` 会**逐一物理归0**（真实运动）；测试可传 `home_on_init=false`
  手动控制时序。

## 测试

| 用例 | 内容 | 状态 |
| :-- | :-- | :-- |
| thread_pool_test | 100 任务全执行 / 并发活跃>1 / 0 线程池 submit false | ✅ 通过 |
| motor_device_test Phase A | Motor：开闸→正转→停→位置45°→回0→锁闸（验证过滤+三环） | ✅ 实机通过 |
| motor_device_test Phase B | MultiMotor：构造归0→submit_speed→角度变化确认→回0→锁闸 | ✅ 实机通过（速度指令后 -0.0°→27.2°，线程池发布有效） |

> 实机注意：Phase A/B 各用独立 CanComm，前一阶段销毁后才开下一阶段，避免同总线
> 双 socket 串扰。与 GUI 同样不能同时运行。
