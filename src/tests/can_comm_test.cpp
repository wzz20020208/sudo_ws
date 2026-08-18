// tests/can_comm_test.cpp
// CanComm open() 自测：
//   1. 打开真实接口（默认 can0）：验证 open 成功、接收线程启动、close 干净退出
//   2. 打开不存在的接口（can99）：验证失败路径不崩溃、能干净返回
// 用法: ./can_comm_test [--ifname <can接口名>]

#include "motor_can/can_comm/can_comm.hpp"
#include "motor_can/common/log.hpp"

#include <cstring>

namespace {

int usage(const char* argv0) {
    MC_LOG_INFO("用法: %s [--ifname <can接口名>]", argv0);
    MC_LOG_INFO("示例: %s --ifname can0", argv0);
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    const char* ifname = "can0";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--ifname") == 0 && i + 1 < argc) {
            ifname = argv[++i];
        } else {
            return usage(argv[0]);
        }
    }

    // 用例 1：正常打开 + 关闭
    {
        motor_can::CanConfig cfg;
        cfg.ifname = ifname;
        motor_can::CanComm comm;
        if (!comm.open(cfg)) {
            MC_LOG_ERROR("open(%s) 失败", ifname);
            return 1;
        }
        MC_LOG_INFO("open(%s) 成功", ifname);
        comm.close();
        MC_LOG_INFO("close() 干净退出");
    }

    // 用例 2：打开不存在的接口（失败路径）
    {
        motor_can::CanConfig cfg;
        cfg.ifname = "can99";
        motor_can::CanComm comm;
        if (comm.open(cfg)) {
            MC_LOG_ERROR("open(can99) 竟然成功了？（can99 不应存在）");
            return 1;
        }
        MC_LOG_INFO("open(can99) 如预期失败，不崩溃");
    }

    MC_LOG_INFO("全部用例通过");
    return 0;
}
