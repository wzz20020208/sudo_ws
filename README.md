# motor_can

通过 **CAN 协议控制脉塔（MYACTUATOR）RH 系列 / RMD-X 兼容谐波关节模组**的 C++ 项目。

- 语言/构建：C++17，CMake（≥ 3.16），零第三方运行依赖
- 运行平台：Linux（Orange Pi，aarch64），SocketCAN（gs_usb / canable.io 适配器实测）
- 分层架构：Transport（CAN 收发）→ Protocol（RH 指令编解码）→ Device（电机控制）→ App（GUI/逻辑）
- 里程碑 1~3 已实机验证：传输层、协议层、Device 层（单机三环 + 多机线程池）

## 快速开始

```bash
# 1. 配置 CAN 接口（1Mbps，RH 系列要求）
sudo bash tools/can_setup.sh --ifname can0 --bitrate 1000000 --loopback off

# 2. 构建
cd build
cmake ..
make -j
```

> 无硬件自测传输通路可用 `--loopback on`。

---

## 用户指南：初始化电机

### 硬件前提

- 电机 **24V 供电**（RH 手册标 48V，本工程实测 X2-7 用 24V）
- 总线上每台电机 **CAN ID 唯一**（1~32），用上位机设好
- `can0` 已按上面步骤 up，程序运行于同一主机的普通用户（非 root 亦可）

### 单机初始化

`Motor` 构造函数即完成「CAN 连接 + 归0」——`home_on_init=true` 时自动**开闸并物理归0**（发位置指令到 0°，真实运动）：

```cpp
#include "motor_can/can_comm/can_comm.hpp"
#include "motor_can/motor/motor.hpp"

using namespace motor_can;

CanConfig cfg;
cfg.ifname = "can0";            // SocketCAN 接口名
CanComm comm;
if (!comm.open(cfg)) { /* 失败处理 */ }

Motor::Config mcfg;
mcfg.max_speed_dps = 15;        // 位置环默认限速（°/s）
mcfg.max_torque_pct = 30;       // 速度环默认限扭（额定电流 %）
mcfg.home_on_init  = true;      // 构造时开闸 + 归0（真实运动）
Motor motor(comm, 1, mcfg);     // 初始化完成：连接已建立，电机已归0
```

不需要归0（例如要手动控制时序）就传 `home_on_init = false`。

### 多机初始化

多机**共享一个 CanComm**，每台电机逐台添加（添加即归0），控制指令由线程池异步发布：

```cpp
#include "motor_can/motor/multi_motor.hpp"

MultiMotorController ctrl(cfg, 4);   // 开共享总线 + 4 线程池
if (!ctrl.is_open()) { /* 失败处理 */ }

ctrl.add_motor(1);                   // 添加电机 1（默认 Config，添加即归0）
ctrl.add_motor(2);                   // 添加电机 2（物理 ID=2）
// 每台可独立配置：ctrl.add_motor(2, motor_can::Motor::Config{...});
```

---

## 用户指南：控制电机

### 单机三环控制

接口返回 `bool`（成功与否），并可回读控制命令回复的运行状态（温度/电流/转速/角度）：

```cpp
// 带抱闸电机，运动前先开闸
motor.brake_release();

MotorRunStatus rs;
motor.set_speed(10.0, &rs);         // 速度环：目标 10°/s（默认限扭 30%）
motor.set_speed(10.0, 50, &rs);     // 速度环 + 指定限扭 50%
motor.set_position(45.0, &rs);      // 位置环：到 45°（默认限速 15°/s）
motor.set_position(-30.0, 20, &rs); // 位置环 + 指定限速 20°/s
motor.set_current(0.5, &rs);        // 转矩环：目标电流 0.5A

motor.stop();                       // 0x81 停止保持
motor.brake_lock();                 // 0x78 锁闸（带抱闸电机停止后再锁）
```

### 状态读取

```cpp
MotorStatus st;
motor.read_status(st);              // 0x9A 温度/电压/抱闸/错误
MotorRunStatus rs;
motor.read_run_status(rs);          // 0x9C 转速/电流/角度
double angle;
motor.read_angle(angle);            // 0x92 多圈绝对角度（°）
```

### 多机批量控制（异步发布）

`submit_*` 入队即返回，线程池工作线程异步发布到对应电机；不同电机并行、同一电机串行：

```cpp
ctrl.submit_speed(1, 10.0, 30);      // 电机1 转 10°/s，限扭 30%
ctrl.submit_position(2, 90.0, 20);   // 电机2 到 90°，限速 20°/s
ctrl.submit_stop(1);
ctrl.submit_brake(1, false);         // 锁闸
ctrl.submit_home(2);                 // 归0

// 同步读取（监控/测试用）
double a;
ctrl.read_angle(1, a);
```

> 所有控制指令的**回复过滤已内建**：发指令后循环收直到命令字节匹配，杂帧自动丢弃——
> 即使总线上有其他程序在轮询，单机 `Motor` 也不会收到错帧。

---

## 验证工具（测试程序）

| 程序 | 用途 | 是否需硬件 |
| :-- | :-- | :-- |
| `./build/protocol_test` | 协议层编解码 44 例（与手册逐字节比对） | 否 |
| `./build/thread_pool_test` | 线程池逻辑 | 否 |
| `./build/can_comm_test` | 传输层 open/send/receive | 可 loopback |
| `./build/motor_spin_test` | 协议层实机：慢速正反转 + 状态读取 | 是 |
| `./build/motor_device_test` | Device 层实机：Phase A 单机三环 / Phase B 多机冒烟 | 是 |

```bash
./build/motor_device_test [--ifname can0] [--id 1]   # 实机，会真实驱动电机
```

## 使用示例（src/examples/）

编译好的最小参考程序，展示 Device 层抽象接口的标准用法：单机走 `Motor`，多机走
`MultiMotorController::submit_*` 批量发布（不使用 0x280 广播）：

```bash
cd build
cmake ..
make -j example_single_motor example_multi_motor example_follow_demo
```

| 程序 | 内容 |
| :-- | :-- |
| `./build/example_single_motor [ifname]` | 单机：读状态 → 开闸 → 位置到 90° → 回读角度 → 停止锁闸 |
| `./build/example_multi_motor [ifname]` | 多机：批量位置（ID1→90°、ID2→180°）→ 逐台回读 → 批量停止 |
| `./build/example_follow_demo [ifname]` | 主从跟随：电机1 由外部控制（本程序只读其角度），电机2 实时跟随（不限速，按回车停止） |

> 示例会真实驱动电机，运行前确认关节活动范围无人。源码：
> `src/examples/single_motor/main.cpp`、`src/examples/multi_motor/main.cpp`、
> `src/examples/follow_demo/main.cpp`。

## 添加自己的程序（CMakeLists.txt）

按层次链接对应库即可（`target_link_libraries`），依赖会自动带上：

| 库 | 内容 | 头文件 |
| :-- | :-- | :-- |
| `motor_can_can_comm` | 传输层：CanComm 收发 | `motor_can/can_comm/can_comm.hpp` |
| `motor_can_protocol` | 协议层：RH 指令编解码 | `motor_can/protocol/rh_protocol.hpp` |
| `motor_can_motor` | Device 层：Motor / MultiMotorController | `motor_can/motor/...` |

在 `CMakeLists.txt` 里加一个不依赖 Qt 的程序（源码放 `src/examples/` 等任意位置）：

```cmake
add_executable(my_app src/my_app.cpp)
target_link_libraries(my_app PRIVATE motor_can_motor)  # 用到 Device 层就链这一个，传输层/协议层自动带上
```

需要 Qt5 Widgets 的 GUI 程序（源码放 `src/gui/`，参考 `src/gui/motor_full_control/`）：

```cmake
find_package(Qt5 COMPONENTS Widgets QUIET)
if(Qt5Widgets_FOUND)
    add_executable(my_gui src/gui/my_gui/main.cpp)
    target_link_libraries(my_gui PRIVATE motor_can_motor Qt5::Widgets)
endif()
```

加完重新配置再编译（**改过 CMakeLists.txt 必须先 `cmake ..` 重新生成**）：

```bash
cd build
cmake ..
make -j my_app          # 或 make -j 全量编译
```

> 顶层已全局开 `-Wall -Wextra -Wpedantic`，新 target 自动带上；本项目 GUI 均不用
> `Q_OBJECT`/AUTOMOC（QTimer 主线程轮询 + connect lambda），无需额外开启。

## GUI 前端（Qt5，可选）

| 程序 | 功能 |
| :-- | :-- |
| `./build/motor_monitor` | 状态监测（0x9A/0x9C）+ 三环 PID 配置 + 多圈零点设置 |
| `./build/motor_position_control` | 位置闭环控制（目标角度/回0/开闸/停止/锁闸） |
| `./build/motor_full_control` | 全量控制（单窗口六 tab：三环/单圈/增量/力控、状态、PID/加速度、编码器零点、0x20 系统、多机广播） |

```bash
sudo ./build/motor_monitor [--ifname can0]
sudo ./build/motor_position_control [--ifname can0]
```

## 目录结构

```
sudo_ws/
├── CMakeLists.txt
├── include/motor_can/
│   ├── can_comm/        # 传输层：CanComm / CanConfig / CanFrame
│   ├── protocol/        # 协议层：rh_protocol.hpp（RH 指令编解码）
│   ├── motor/           # Device 层：Motor（单机）/ MultiMotorController（多机）
│   └── common/          # ThreadPool / log
├── src/                 # 全部源码：库 + GUI + 测试 + 示例
│   ├── can_comm/  protocol/  motor/  common/   # 各层库实现（与 include 同名一一对应）
│   ├── gui/              # Qt5 前端（motor_monitor / motor_position_control / motor_full_control）
│   ├── tests/            # 测试程序（protocol/thread_pool/can_comm/motor_spin/motor_device）
│   └── examples/         # 使用示例（single_motor / multi_motor / follow_demo）
├── tools/can_setup.sh   # 配置 CAN 接口
├── docs/                # 设计文档（01 架构 / 02 传输层 / 03 协议层 / 04 Device层 / 05 跟随demo分析）
├── third_party/         # 第三方代码与参考（myactuator_rmd-* / reference）
└── docs_cn/             # 电机厂商资料
```

## 注意事项

- **监控可与控制共存，但两个「发运动指令」的程序不能同时跑**：`motor_monitor` 是只读监控（Device 层过滤读接口），可一直开着与任何控制程序共存；若两个程序都往同一台电机发运动指令，则会互相覆盖
- 位置/速度指令会**真实驱动电机**，运行前确认关节活动范围内无人、无障碍
- 带抱闸电机：运动前 `brake_release()`，运动完停下后再 `brake_lock()`

## 里程碑进度

| 里程碑 | 内容 | 状态 |
| :-- | :-- | :-- |
| 1 | 传输层：SocketCAN 收发（open/send/receive/receive_by_id） | ✅ 实机通过 |
| 2 | 协议层：RH 指令编解码 + 单位换算（三环/PID/加速度/编码器/模式/增量·力控位置） | ✅ 实机通过 |
| 3 | Device 层：单机三环 + 多机线程池发布 + 请求/回复过滤 | ✅ 实机通过 |
| 4 | App 层：控制逻辑整合 + GUI 改用 Device 层 + 整机联调 | ⏳ |

详见 [docs/01_总体架构.md](docs/01_总体架构.md) 与各层设计文档。
