# CAN 协议层设计（里程碑 2）

## 目标

提供 RH 协议（《伺服电机控制协议 V4.3》，适用驱动 V3）单电机指令的**编解码与单位换算**。
与 X2-7 关节模组实测兼容。

- 只回答「某个指令对应的帧长什么样」，不做收发——发送 / 收回复由调用方用 CanComm
  （`send` + `receive_by_id`）完成。
- 全部为单电机指令：指令 `0x140+ID`，回复 `0x240+ID`，标准帧，DLC=8。

## 目录与文件

```
include/motor_can/protocol/rh_protocol.hpp   # 编解码函数声明 + 数据结构（已过目）
src/protocol/rh_protocol.cpp                 # 实现（已过目）
src/tests/protocol_test.cpp                  # 纯编解码自测（不碰硬件，24 例通过）
```

CMake 目标：`motor_can_protocol`（静态库，`PUBLIC` 依赖 `motor_can_can_comm`）。

## 接口形态

自由函数 + 具名命令枚举，无类、无状态：

```cpp
enum class RhCmd : uint8_t { SystemReset=0x76, BrakeRelease=0x77, BrakeLock=0x78,
                             MotorOff=0x80, MotorStop=0x81, ReadAngle=0x92,
                             ReadStatus=0x9A, ReadStatus2=0x9C, Torque=0xA1,
                             Speed=0xA2, Position=0xA4 };

struct MotorStatus { int8_t temp_c; int8_t mos_temp_c; bool brake_released;
                     double voltage_v; uint16_t error_state; };        // 0x9A
struct MotorRunStatus { int8_t temp_c; double iq_a; double speed_dps; double angle_deg; };

// 发方向：组帧（物理单位 -> 帧）
CanFrame encode_command(uint8_t id, RhCmd cmd);                      // 单字节命令 / 只读查询
CanFrame encode_speed(uint8_t id, double speed_dps, uint8_t max_torque_pct);   // 0xA2
CanFrame encode_torque(uint8_t id, double current_a);                          // 0xA1
CanFrame encode_position(uint8_t id, double angle_deg, uint16_t max_speed_dps);// 0xA4

// 收方向：解帧（帧 -> 物理单位），命令字节不符 / DLC<8 返回 false
bool decode_status(const CanFrame& reply, MotorStatus& out);          // 0x9A
bool decode_run_status(const CanFrame& reply, MotorRunStatus& out);   // 0x9C + 控制命令回复
bool decode_angle(const CanFrame& reply, double& angle_deg);          // 0x92
```

## 关键设计决策

- **解码输出物理单位**：不吐原始 LSB，单位换算集中在协议层（V / A / °/s / °）。
- **一个 `decode_run_status` 通吃 0x9C 和所有控制命令回复**：0xA1/0xA2/0xA4（以及
  未来的 0xA8/0xA9）的回复布局与 0x9C 完全相同——Data[1]温度、Data[2..3]转矩电流
  (0.01A)、Data[4..5]转速(1°/s)、Data[6..7]角度(1°)，故只校验命令字节集合即共享解码。
- **`enum class RhCmd` 类型安全**：`encode_command` 不接受任意字节；命令字节具名，
  满足「魔法数字具名」要求。
- **小端序打包/解包**：匿名命名空间 `put_le16/put_le32`、`get_le16/get_le32`。
- **超范围钳位**：`encode_*` 无错误通道（返回 CanFrame），物理值换算后超出目标位宽时
  钳位到 `INT16/INT32` 边界（如 0xA1 电流钳到 ±327.67A）。
- **不校验 `id`**（须 1~32，调用方职责），与 CanComm 不校验帧内容一致。

## 命令集范围（第一批：核心命令）

| 类别 | 命令 | 组帧 | 解帧 |
| :-- | :-- | :-- | :-- |
| 抱闸/停止/关闭/复位 | 0x77 0x78 0x80 0x81 0x76 | encode_command | 无（0x76 无回复，其余回显同帧） |
| 只读查询 | 0x92 0x9A 0x9C | encode_command | decode_angle / decode_status / decode_run_status |
| 控制 | 0xA1 0xA2 0xA4 | encode_speed / encode_torque / encode_position | decode_run_status |

## 命令集范围（第二批：参数 / 编码器 / 更多控制）

| 类别 | 命令 | 组帧 | 解帧 |
| :-- | :-- | :-- | :-- |
| PID 读写 | 0x30 0x31 0x32 | encode_read_pid / encode_write_pid_ram / encode_write_pid_rom | decode_pid |
| 加速度读写 | 0x42 0x43 | encode_read_accel / encode_write_accel | decode_accel |
| 编码器 | 0x60 0x61 0x62 0x63 | encode_command（读）/ encode_write_encoder_offset | decode_encoder_position |
| 单圈角度 / 模式 | 0x94 0x70 | encode_command | decode_single_angle / decode_mode |
| 控制 | 0xA8 0xA9 | encode_increment_position / encode_force_position | decode_run_status |

> 第二批要点：
> - PID 值按 **float32（IEEE754 小端）** 打包在 Data[4..7]，`encode_write_pid_ram/rom`
>   （0x31/0x32）共用内部 `pid_write_frame`；`decode_pid` 通吃 0x30/0x31/0x32。
> - 加速度 0x43 钳位到协议范围 **100~60000** °/s²（uint32 打包）。
> - 0x60/0x61/0x62 及写命令回显 0x63/0x64 回复布局相同（int32@Data[4..7]），
>   `decode_encoder_position` 通吃（与 decode_pid 接受 0x31/0x32 写回显一致）。
> - 0xA8 布局与 0xA4 相同；0xA9 另在 Data[1] 带最大扭矩（额定电流百分比）。
> - 0x94 单圈角度在 Data[6..7]（uint16，0.01°/LSB），与 0x92 的 Data[4..7] 不同。
> - 未做：版本查询（0xB1/0xB2/0xB5）、0x64 零点写（需 0x76 复位才生效，纯单字节，
>   经 encode_command 即可），按需再加。

## 实现进度

| 步骤 | 内容 | 状态 |
| :-- | :-- | :-- |
| Step 1 | 头文件声明（RhCmd / 数据结构 / 编解码函数） | ✅ 已过目 |
| Step 2 | 实现（组帧 + 解帧 + 单位换算 + 钳位） | ✅ 已过目 |
| Step 3 | `protocol_test` 自测（24 例，与手册示例逐字节比对） | ✅ 通过 |
| Step 4 | `motor_spin_test` 改用协议层 API | ✅ 实机通过 |
| Step 5 | 实机验证正反转 | ✅ 正转减速到 8dps、反转达 -10dps，全部指令收到正确回复 |
| Step 6 | 第二批命令（PID/加速度/编码器/增量·力控位置/单圈角度/模式） | ✅ 已过目，44 例自测通过 |

> 实机注意：CAN 总线广播，GUI 与测试**不能同时运行**——后者的 socket 也会收到前者查询的
> 回复，`receive_by_id` 会弹到杂帧（命令字节不符）。过滤（循环收直到匹配期望回复）是
> Device 层（里程碑 3）的请求/回复语义。已知现象：X2-7 抱闸释放后 0x9A Data[3] 仍显示
> 「锁死」，但电机实际已转（状态位不回写）。

> 遵守「不要一步到位」：每个函数实现前确认方案、实现后确认结果。
