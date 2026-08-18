// include/motor_can/motor/can_id.hpp
// 总线级 CAN ID 操作（0x20 功能控制指令，索引 0x05 设置 CANID）。
//
// 与 Motor 单机接口的区别：改 ID 面向「不知道目标电机当前 ID 编号」的场景，
// 独立成文件直接操作 CanComm。0x20 写 ID 按地址 0x140+current_id 发送。
#pragma once

#include "motor_can/can_comm/can_comm.hpp"
#include "motor_can/protocol/rh_protocol.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

namespace motor_can {

// 枚举总线上所有设备：逐 ID 扫描，对每个 ID 发 0x9A 读并等 0x240+id 回复，
// 有回复即该 ID 在线。0x79 广播读在这版固件上无效，改用逐 ID 扫描。
// 每 ID 最多等 per_id_timeout（默认 30ms），全扫最多约 1s。
// ids：在线的 CAN ID 列表（1~32，升序）。返回 false = 总线上没有任何设备。
// 注意：两台电机同 ID 时回复帧完全相同会合并，扫描只能判断「该 ID 有无电机」，
// 无法区分同 ID 的电机会数。
bool enumerate_can_ids(CanComm& comm, std::vector<uint8_t>& ids,
                       std::chrono::milliseconds per_id_timeout = std::chrono::milliseconds(30));

// 按地址写 CAN ID（0x20 索引 0x05，发到 0x140+current_id）。
// current_id：电机当前 CAN ID；new_id：新 CAN ID（1~32）。
// 实测该电机收到写指令后立即切到新 ID 并回显在 0x240+new_id；文档示例按
// 0x240+current_id 回显，故两处都等一次。新 ID 保存到 ROM，需重启电机
// （断电→上电）才持久化，不重启掉电后回退。
// 返回 true = 收到回显且新 ID 一致。
bool write_can_id(CanComm& comm, uint8_t current_id, uint8_t new_id);

}  // namespace motor_can
