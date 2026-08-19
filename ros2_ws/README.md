# ros2_ws —— ROS2 电机状态发布工作空间

ROS2 **Humble** 工作空间（colcon），嵌套在 `sudo_ws/` 下，与上层纯 CMake 工程互不干扰。
包内节点直接链接上层 `sudo_ws/build/` 预编译的 motor_can 静态库 + `sudo_ws/include/`，
不在包内重复编译电机库。

## 首次准备

```bash
# 1. 装 colcon（需 sudo）
sudo apt install -y python3-colcon-common-extensions

# 2. source ROS2
source /opt/ros/humble/setup.bash
```

> **⚠ 必须用系统 python 构建**：本机 PATH 里 miniforge 的 python3 排第一，它缺 ROS2 消息
> 生成所需的 `em`（empy）模块，会报 `ModuleNotFoundError: No module named 'em'`。
> 构建前先把 /usr/bin 提到前面（或临时退出 conda）：
> ```bash
> export PATH=/usr/bin:/bin:$PATH
> ```

## 构建与运行

```bash
cd /home/orangepi/sudo_ws/ros2_ws
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:/bin:$PATH
colcon build                      # 只构建 src/ 下的包
source install/setup.bash

# 运行：默认 can0 / 电机 ID 1
ros2 run motor_status_publisher motor_status_publisher_node --ifname can0 --id 1
```

另开终端验证：
```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/sudo_ws/ros2_ws/install/setup.bash
ros2 topic echo /motor_status
```

## 包：motor_status_publisher

- 话题 `/motor_status`，自定义 msg `MotorStatus`（`std_msgs/Header + angle_deg + speed_dps + iq_a`）
- 节点 100Hz 定时读 **0x9C** 发布角度/转速/转矩电流（`iq_a` 与输出力矩成正比）；
  读失败发上次已知值保持频率，`Motor` 为只读句柄（`home_on_init=false`，`reply_timeout=9ms`）
- CAN 前置：`sudo ip link set can0 type can bitrate 1000000 && up`
  （双适配器见上层 README 注意事项：120Ω 只能总线两端各一个、用 `--ifname` 选接口）
