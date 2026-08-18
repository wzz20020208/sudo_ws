// src/motor/multi_motor.cpp
// MultiMotorController 多机控制实现：批量提交控制指令，线程池异步发布到对应 Motor。

#include "motor_can/motor/multi_motor.hpp"

#include "motor_can/common/log.hpp"

#include <algorithm>
#include <utility>

namespace motor_can {

namespace {
// 未管理的电机 ID 统一告警并拒绝
bool log_unknown_id(uint8_t id) {
    MC_LOG_ERROR("MultiMotorController: 电机 %u 未在管理集合中", id);
    return false;
}
}  // namespace

MultiMotorController::MultiMotorController(const CanConfig& can_config, size_t pool_threads)
    : pool_(pool_threads) {
    if (!comm_.open(can_config)) {
        MC_LOG_ERROR("MultiMotorController: open(%s) 失败", can_config.ifname.c_str());
        return;
    }
    MC_LOG_INFO("MultiMotorController: 总线已打开（%s），线程池 %zu 线程，等待 add_motor",
                can_config.ifname.c_str(), pool_threads);
}

bool MultiMotorController::add_motor(uint8_t id, const Motor::Config& config) {
    if (!comm_.is_open()) {
        MC_LOG_ERROR("MultiMotorController: 总线未打开，无法添加电机 %u", id);
        return false;
    }
    const auto [it, inserted] =
        motors_.emplace(id, std::make_unique<Motor>(comm_, id, config));
    if (!inserted) {
        MC_LOG_ERROR("MultiMotorController: 电机 %u 已在管理集合中，跳过", id);
        return false;
    }
    MC_LOG_INFO("MultiMotorController: 已添加电机 %u", id);
    return true;
}

bool MultiMotorController::add_motor(uint8_t id) {
    return add_motor(id, Motor::Config());
}

std::vector<uint8_t> MultiMotorController::ids() const {
    std::vector<uint8_t> out;
    out.reserve(motors_.size());
    for (const auto& [id, motor] : motors_) {
        out.push_back(id);
    }
    return out;
}

Motor* MultiMotorController::find(uint8_t id) {
    const auto it = motors_.find(id);
    return it == motors_.end() ? nullptr : it->second.get();
}

bool MultiMotorController::submit_current(uint8_t id, double current_a) {
    Motor* m = find(id);
    if (m == nullptr) {
        return log_unknown_id(id);
    }
    return pool_.submit([m, current_a] {
        MotorRunStatus st;
        m->set_current(current_a, &st);
    });
}

bool MultiMotorController::submit_speed(uint8_t id, double speed_dps, uint8_t max_torque_pct) {
    Motor* m = find(id);
    if (m == nullptr) {
        return log_unknown_id(id);
    }
    return pool_.submit([m, speed_dps, max_torque_pct] {
        MotorRunStatus st;
        m->set_speed(speed_dps, max_torque_pct, &st);
    });
}

bool MultiMotorController::submit_position(uint8_t id, double angle_deg, uint16_t max_speed_dps) {
    Motor* m = find(id);
    if (m == nullptr) {
        return log_unknown_id(id);
    }
    return pool_.submit([m, angle_deg, max_speed_dps] {
        MotorRunStatus st;
        m->set_position(angle_deg, max_speed_dps, &st);
    });
}

bool MultiMotorController::submit_stop(uint8_t id) {
    Motor* m = find(id);
    if (m == nullptr) {
        return log_unknown_id(id);
    }
    return pool_.submit([m] { m->stop(); });
}

bool MultiMotorController::submit_brake(uint8_t id, bool release) {
    Motor* m = find(id);
    if (m == nullptr) {
        return log_unknown_id(id);
    }
    return pool_.submit([m, release] {
        if (release) {
            m->brake_release();
        } else {
            m->brake_lock();
        }
    });
}

bool MultiMotorController::submit_home(uint8_t id) {
    Motor* m = find(id);
    if (m == nullptr) {
        return log_unknown_id(id);
    }
    return pool_.submit([m] { m->home(); });
}

bool MultiMotorController::broadcast_stop() {
    if (!comm_.is_open()) {
        MC_LOG_ERROR("MultiMotorController: 总线未打开，无法广播 0x81 停止");
        return false;
    }
    // id 参数只影响单机帧 ID（0x140+id），to_broadcast 会改写成 0x280，数据域不受影响
    return comm_.send(to_broadcast(encode_command(1, RhCmd::MotorStop)));
}

bool MultiMotorController::broadcast_off() {
    if (!comm_.is_open()) {
        MC_LOG_ERROR("MultiMotorController: 总线未打开，无法广播 0x80 关闭");
        return false;
    }
    return comm_.send(to_broadcast(encode_command(1, RhCmd::MotorOff)));
}

bool MultiMotorController::broadcast_position(double angle_deg, uint16_t max_speed_dps) {
    if (!comm_.is_open()) {
        MC_LOG_ERROR("MultiMotorController: 总线未打开，无法广播 0xA4 位置");
        return false;
    }
    return comm_.send(to_broadcast(encode_position(1, angle_deg, max_speed_dps)));
}

bool MultiMotorController::read_status(uint8_t id, MotorStatus& out) {
    Motor* m = find(id);
    if (m == nullptr) {
        return log_unknown_id(id);
    }
    return m->read_status(out);
}

bool MultiMotorController::read_angle(uint8_t id, double& angle_deg) {
    Motor* m = find(id);
    if (m == nullptr) {
        return log_unknown_id(id);
    }
    return m->read_angle(angle_deg);
}

}  // namespace motor_can
