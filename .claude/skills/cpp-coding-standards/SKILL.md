---
name: cpp-coding-standards
description: 本项目 C++ 编码规范（命名规范 + 编写规范）。编写、审查或修改本仓库任何 C++ 代码时先加载本技能并严格遵循。
---

# C++ 编码规范（motor_can 项目）

适用本仓库所有 C++ 代码。语言标准 C++17，构建 CMake（≥ 3.16），零第三方运行依赖。

## 一、命名规范

| 类别 | 规则 | 示例 |
| :-- | :-- | :-- |
| 命名空间 | 全小写 | `motor_can` |
| 类 / 结构体 / 枚举类 | PascalCase | `CanFrame`、`CanComm`、`CanError` |
| 函数 / 成员函数 | snake_case | `send`、`receive_by_id`、`is_open` |
| 局部变量 / 参数 | snake_case | `frame`、`timeout_ms`、`config` |
| 成员变量 | snake_case + 尾下划线 `_` | `fd_`、`rx_thread_`、`rx_queues_` |
| 常量（constexpr/const） | `k` + PascalCase | `kMaxQueueDepth`、`kDefaultBitrate` |
| 枚举值 | PascalCase | `CanError::ReceiveTimeout` |
| 宏 | `MC_` 前缀 + 全大写 | `MC_LOG_INFO` |
| 文件 | snake_case | `can_comm.hpp` / `can_comm.cpp` |
| 头文件保护 | `#pragma once` | — |

### 文件与目录布局

- **按「电机控制视角」分文件夹**，`include/` 与 `src/` 同名一一对应：
  - `include/motor_can/can_comm/` ↔ `src/can_comm/` —— CAN 底层通讯（CanComm）
  - `include/motor_can/protocol/`  ↔ `src/protocol/`  —— 电机协议（RH 指令编解码）
  - `include/motor_can/motor/`     ↔ `src/motor/`     —— 电机控制（闭环）
  - `include/motor_can/common/`    ↔ `src/common/`    —— 公共工具
- 头文件：`include/motor_can/<模块>/xxx.hpp`
- 实现：`src/<模块>/xxx.cpp`，与头文件目录结构一一对应
- 一个类一对文件（`.hpp` + `.cpp`）；纯类型可合并头文件（如 `can_types.hpp`）
- 目录按电机控制模块划分，而非硬件类型：将来接入 USB-CAN 只在 `can_comm/` 内加文件

## 二、编写规范

### 语言与风格

- C++17；**不用异常**，错误用返回值（bool / 错误枚举）表达
- 缩进 4 空格，不使用 tab；每行 ≤ 100 列
- 大括号风格：**K&R**（左花括号与声明/语句同行）
- 用空行把代码分成逻辑块

### 头文件

- 自包含（单独 include 能编译）；一律 `#pragma once`
- include 顺序：本项目头 → 标准库/系统头，组间空行，组内字母序
- 公共接口写中文注释：说明**职责**与**返回值语义**；语义不显然的参数逐条注释

### 资源与内存

- RAII 管理一切资源（文件描述符、线程、锁）
- 禁止裸 `new`/`delete`、禁止裸指针所有权
- 成员函数若不会抛错、不会阻塞，标记 `noexcept`

### 并发

- 共享状态一律由 `std::mutex`（+ `std::condition_variable`）保护
- 线程生命周期：负责启动的人负责 `join()`；停止时先置停止标志再解阻塞，避免线程悬挂

### 错误处理

- 失败返回 `false` / 错误枚举；**不静默吞错**，调用方必须检查返回值
- 底层错误可用日志（`MC_LOG_*`）记录现场后返回

### 注释

- 中文注释；解释**为什么**，不解释是什么
- 魔法数字必须注释或提为具名常量

## 三、收尾检查（提交 / 审查前）

- [ ] 所有公共符号符合命名规范
- [ ] 无警告编译（`-Wall -Wextra -Wpedantic`）
- [ ] 无裸指针 / 裸 new/delete
- [ ] 头文件自包含且 `#pragma once`
- [ ] 每个失败分支都处理或显式记录
