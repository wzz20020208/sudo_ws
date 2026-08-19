cmake_minimum_required(VERSION 3.8)
project(motor_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 依赖：motor_status_publisher 提供 /motor_status 的消息类型（消费方）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_status_publisher REQUIRED)
find_package(Threads REQUIRED)

# motor_can 预编译静态库（sudo_ws/build），在此只链接不重复编译
set(MOTOR_CAN_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../build")
set(MOTOR_CAN_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../include")

add_executable(motor_follower_node src/motor_follower_node.cpp)
target_include_directories(motor_follower_node PRIVATE ${MOTOR_CAN_INC_DIR})
# 消费 motor_status_publisher 的 msg：链接其 C++ 类型支持目标（find_package 导出的名字带
# motor_status_publisher:: 命名空间；注意 plain 签名，不带 PRIVATE，与 ament_target_dependencies
# 内部签名一致，否则 CMake 报混用错误）
target_link_libraries(motor_follower_node
  motor_status_publisher::motor_status_publisher__rosidl_typesupport_cpp
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_motor.a
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_protocol.a
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_can_comm.a
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_common.a
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_core.a
  Threads::Threads
)
ament_target_dependencies(motor_follower_node rclcpp std_msgs motor_status_publisher)

install(TARGETS motor_follower_node DESTINATION lib/${PROJECT_NAME})

ament_package()
