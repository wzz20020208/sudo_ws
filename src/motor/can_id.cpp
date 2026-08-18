// src/motor/can_id.cpp
// 总线级 CAN ID 操作实现（0x20 功能控制，索引 0x05 设置 CANID）：
//  - enumerate_can_ids：逐 ID 扫描，发 0x9A 读收集在线 ID；
//  - write_can_id：按地址写 CAN ID，等回显确认。

#include "motor_can/motor/can_id.hpp"

namespace motor_can {

bool enumerate_can_ids(CanComm& comm, std::vector<uint8_t>& ids,
                       std::chrono::milliseconds per_id_timeout) {
    ids.clear();
    for (uint8_t id = 1; id <= 32; ++id) {
        if (!comm.send(encode_command(id, RhCmd::ReadStatus))) {
            return false;
        }
        CanFrame reply;
        if (comm.receive_by_id(0x240u + id, reply, per_id_timeout) &&
            reply.dlc >= 8 && reply.data[0] == static_cast<uint8_t>(RhCmd::ReadStatus)) {
            ids.push_back(id);
        }
    }
    return !ids.empty();
}

bool write_can_id(CanComm& comm, uint8_t current_id, uint8_t new_id) {
    if (!comm.send(encode_set_can_id(current_id, new_id))) {
        return false;
    }
    // 实测 X2-7 收到后立即切到新 ID 并回显在 0x240+new_id；文档示例按 0x240+current_id
    // 回显，两处都等一次兜底
    CanFrame reply;
    if (comm.receive_by_id(0x240u + new_id, reply, std::chrono::milliseconds(300))) {
        uint8_t got = 0;
        return decode_set_can_id(reply, got) && got == new_id;
    }
    if (comm.receive_by_id(0x240u + current_id, reply, std::chrono::milliseconds(300))) {
        uint8_t got = 0;
        return decode_set_can_id(reply, got) && got == new_id;
    }
    return false;
}

}  // namespace motor_can
