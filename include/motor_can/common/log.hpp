// include/motor_can/common/log.hpp
// 轻量 header-only 日志，无第三方依赖。
// 编译期可用 -DMOTOR_CAN_LOG_LEVEL=motor_can::LogLevel::Info 调整输出级别，
// 默认 Debug 全量输出。
#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <ctime>

namespace motor_can {

enum class LogLevel : uint8_t {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    None = 4,
};

#ifndef MOTOR_CAN_LOG_LEVEL
#define MOTOR_CAN_LOG_LEVEL motor_can::LogLevel::Debug
#endif

namespace detail {

inline const char* levelName(LogLevel lv) {
    switch (lv) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO ";
        case LogLevel::Warn:  return "WARN ";
        case LogLevel::Error: return "ERROR";
        default:              return "?    ";
    }
}

inline void logWrite(LogLevel lv, const char* fmt, ...) {
    char ts[32];
    const std::time_t now = std::time(nullptr);
    std::tm tmv{};
    localtime_r(&now, &tmv);
    std::strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);

    std::fprintf(stderr, "[%s] %s ", ts, levelName(lv));
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
    std::fprintf(stderr, "\n");
}

}  // namespace detail
}  // namespace motor_can

#define MC_LOG_DEBUG(...)                                                         \
    do {                                                                          \
        if (motor_can::LogLevel::Debug >= MOTOR_CAN_LOG_LEVEL)                    \
            motor_can::detail::logWrite(motor_can::LogLevel::Debug, __VA_ARGS__); \
    } while (0)

#define MC_LOG_INFO(...)                                                         \
    do {                                                                         \
        if (motor_can::LogLevel::Info >= MOTOR_CAN_LOG_LEVEL)                    \
            motor_can::detail::logWrite(motor_can::LogLevel::Info, __VA_ARGS__); \
    } while (0)

#define MC_LOG_WARN(...)                                                         \
    do {                                                                         \
        if (motor_can::LogLevel::Warn >= MOTOR_CAN_LOG_LEVEL)                    \
            motor_can::detail::logWrite(motor_can::LogLevel::Warn, __VA_ARGS__); \
    } while (0)

#define MC_LOG_ERROR(...)                                                         \
    do {                                                                          \
        if (motor_can::LogLevel::Error >= MOTOR_CAN_LOG_LEVEL)                    \
            motor_can::detail::logWrite(motor_can::LogLevel::Error, __VA_ARGS__); \
    } while (0)
