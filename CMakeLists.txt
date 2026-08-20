cmake_minimum_required(VERSION 3.16)
project(motor_can LANGUAGES CXX)

# ---------------------------------------------------------------------------
# 全局编译配置
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# CAN 底层通讯库（协议层 / 电机层后续并入）
# ---------------------------------------------------------------------------
add_library(motor_can_can_comm STATIC
    src/can_comm/can_comm.cpp
)
target_include_directories(motor_can_can_comm PUBLIC ${CMAKE_SOURCE_DIR}/include)
find_package(Threads REQUIRED)
target_link_libraries(motor_can_can_comm PUBLIC Threads::Threads)

# ---------------------------------------------------------------------------
# 底层通讯自测程序
# ---------------------------------------------------------------------------
add_executable(can_comm_test src/tests/can_comm_test.cpp)
target_link_libraries(can_comm_test PRIVATE motor_can_can_comm)

# ---------------------------------------------------------------------------
# RH 协议层（指令编解码，里程碑 2）
# ---------------------------------------------------------------------------
add_library(motor_can_protocol STATIC
    src/protocol/rh_protocol.cpp
)
target_link_libraries(motor_can_protocol PUBLIC motor_can_can_comm)

# 协议层自测（纯编解码，不碰硬件）
add_executable(protocol_test src/tests/protocol_test.cpp)
target_link_libraries(protocol_test PRIVATE motor_can_protocol)

# 电机缓慢转动测试（协议层 API + 实机验证）
add_executable(motor_spin_test src/tests/motor_spin_test.cpp)
target_link_libraries(motor_spin_test PRIVATE motor_can_protocol)

# ---------------------------------------------------------------------------
# 公共工具库（线程池，里程碑 3）
# ---------------------------------------------------------------------------
add_library(motor_can_common STATIC
    src/common/thread_pool.cpp
)
target_include_directories(motor_can_common PUBLIC ${CMAKE_SOURCE_DIR}/include)

# 线程池自测（纯逻辑，不碰硬件）
add_executable(thread_pool_test src/tests/thread_pool_test.cpp)
target_link_libraries(thread_pool_test PRIVATE motor_can_common)

# ---------------------------------------------------------------------------
# 电机控制层（Device 层，里程碑 3：单机三环 + 多机线程池发布）
# ---------------------------------------------------------------------------
add_library(motor_can_motor STATIC
    src/motor/motor.cpp
    src/motor/multi_motor.cpp
    src/motor/can_id.cpp
)
target_link_libraries(motor_can_motor PUBLIC motor_can_can_comm motor_can_protocol motor_can_common)

# 电机控制实机测试（Motor 三环 + 过滤；MultiMotorController 冒烟）
add_executable(motor_device_test src/tests/motor_device_test.cpp)
target_link_libraries(motor_device_test PRIVATE motor_can_motor)

# ---------------------------------------------------------------------------
# 使用示例（Device 层抽象接口，给用户参考；无需 Qt）
# ---------------------------------------------------------------------------
add_executable(example_single_motor src/examples/single_motor/main.cpp)
target_link_libraries(example_single_motor PRIVATE motor_can_motor)

add_executable(example_multi_motor src/examples/multi_motor/main.cpp)
target_link_libraries(example_multi_motor PRIVATE motor_can_motor)

# 电机2 实时跟随电机1 演示（读电机1真实角度，Device 层抽象接口）
add_executable(example_follow_demo src/examples/follow_demo/main.cpp)
target_link_libraries(example_follow_demo PRIVATE motor_can_motor)

# ---------------------------------------------------------------------------
# 电机状态监测前端（Qt5 Widgets，可选：无 Qt 时其余目标仍可构建）
# 用法: ./motor_monitor [--ifname <can接口名>]
# ---------------------------------------------------------------------------
find_package(Qt5 COMPONENTS Widgets QUIET)
if(Qt5Widgets_FOUND)
    add_executable(motor_monitor
        src/gui/motor_monitor/main.cpp
        src/gui/motor_monitor/monitor_window.cpp
        src/gui/motor_monitor/monitor_window.hpp
    )
    target_link_libraries(motor_monitor PRIVATE motor_can_can_comm motor_can_protocol motor_can_motor Qt5::Widgets)

    # 位置闭环控制前端（0xA4 绝对位置 + 0x92/0x9C 轮询，手动开闸/锁闸）
    # 用法: ./motor_position_control [--ifname <can接口名>]
    add_executable(motor_position_control
        src/gui/motor_position_control/main.cpp
        src/gui/motor_position_control/position_window.cpp
        src/gui/motor_position_control/position_window.hpp
    )
    target_link_libraries(motor_position_control PRIVATE motor_can_can_comm motor_can_protocol Qt5::Widgets)

    # 全量电机控制前端（单窗口 + 六 tab，覆盖协议层全部已实现指令，不动现有两个窗口）
    # 用法: ./motor_full_control [--ifname <can接口名>]
    add_executable(motor_full_control
        src/gui/motor_full_control/main.cpp
        src/gui/motor_full_control/full_control_window.cpp
        src/gui/motor_full_control/full_control_window.hpp
        src/gui/widgets/waveform_view.cpp
        src/gui/widgets/waveform_view.hpp
    )
    # 波形控件在 src/gui/widgets/，加入 include 路径供 #include "gui/widgets/waveform_view.hpp"
    target_include_directories(motor_full_control PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(motor_full_control PRIVATE motor_can_can_comm motor_can_protocol motor_can_motor Qt5::Widgets)

    # 波形控件离线测试（无硬件：生成样例 CSV 验证加载/解包/缩放/回实时/存 PNG）
    add_executable(waveform_view_test
        src/tests/waveform_view_test.cpp
        src/gui/widgets/waveform_view.cpp
        src/gui/widgets/waveform_view.hpp
    )
    target_include_directories(waveform_view_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(waveform_view_test PRIVATE Qt5::Widgets)
endif()
