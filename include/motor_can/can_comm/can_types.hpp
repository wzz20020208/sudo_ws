// include/motor_can/can_comm/can_types.hpp
// CAN 底层通讯公共类型：帧、配置、错误码。
//
// 设计要点：不直接暴露 Linux `struct can_frame`，而是自建类型。
// 好处是 SocketCAN / USB-CAN 各实现的收发结构都能翻译成本类型，
// 上层（协议层、设备层）只依赖这里的类型，不感知底层硬件差异。
#pragma once

#include <cstdint>
#include <string>

namespace motor_can {

/// 一帧 CAN 报文。
struct CanFrame {
    uint32_t id;            ///< 帧 ID（不含 EFF/RTR 标志位）
    bool     is_extended;   ///< true=扩展帧(29bit)，false=标准帧(11bit)
    uint8_t  dlc;           ///< 数据长度 0..8
    uint8_t  data[8];       ///< 数据字节
};

/// CAN 接口配置。
struct CanConfig {
    std::string ifname = "can0";    ///< SocketCAN 接口名（如 can0）
    uint32_t    bitrate = 1000000;  ///< 波特率（bps）；RH 电机要求 1Mbps
    bool        loopback = false;   ///< true=接收自己发送的帧（自测用）；真实电机应保持 false
};

/// CAN 错误码，供调用方诊断失败原因。
enum class CanError {
    None,           ///< 无错误
    SendFailed,     ///< 发送失败
    ReceiveTimeout, ///< 接收超时
    BusOff,         ///< 总线关闭（严重错误）
    Closed,         ///< 接口已关闭
    Unknown,        ///< 未知错误
};

}  // namespace motor_can
