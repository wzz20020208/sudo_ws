cmake_minimum_required(VERSION 3.8)
project(motor_status_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

# 自定义消息：电机运行状态
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  DEPENDENCIES std_msgs
)

# motor_can 预编译静态库（sudo_ws/build），在此只链接不重复编译
set(MOTOR_CAN_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../build")
set(MOTOR_CAN_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../include")

add_executable(motor_status_publisher_node src/motor_status_publisher_node.cpp)
target_include_directories(motor_status_publisher_node PRIVATE ${MOTOR_CAN_INC_DIR})
# ${PROJECT_NAME}__rosidl_typesupport_cpp 为本包生成的 msg C++ 类型支持目标。
# 注意：ament_target_dependencies 内部用 plain 签名调 target_link_libraries，
# 此处必须同样用 plain 签名（不能带 PRIVATE 关键字），否则 CMake 报混用错误。
target_link_libraries(motor_status_publisher_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_motor.a
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_protocol.a
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_can_comm.a
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_common.a
  ${MOTOR_CAN_LIB_DIR}/libmotor_can_core.a
  Threads::Threads
)
ament_target_dependencies(motor_status_publisher_node rclcpp std_msgs)

install(TARGETS motor_status_publisher_node DESTINATION lib/${PROJECT_NAME})

ament_package()
