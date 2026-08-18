# 📘 Ray Studio 完整使用指南

**Ray Studio** 是基于 VUE 平台的 EtherCAT 电机控制系统，提供电机控制、PID 调参、工作日志、Topic Terminal 与系统诊断功能。

---

## 📑 目录

- [系统启动](#系统启动)
- [Web-界面概览](#web-界面概览)
- [主要功能模块详解](#主要功能模块详解)
- [系统设置](#系统设置)
- [常见问题解答](#常见问题解答)

---

## 🚀 系统启动

> **默认行为说明**：Ray Studio 出厂默认配置为 **开机自动启动**。  
> 如需进行手动调试、逐项排查或避免开机自动拉起服务，请先查看本节后面的 **“开机默认启动网页服务”**，按说明临时关闭开机自启后再调试。

### 启动步骤

#### 1. 推荐方式：一键启动
```bash
cd ~/ethercat_control_web
./start_all.sh
```

该脚本会自动启动以下服务：
- `rosbridge_server`
- 主后端 API：`backend/services/motor_control_server.py`
- 前端开发服务：`npm run dev -- --host`

常用配套脚本：
```bash
cd ~/ethercat_control_web
./status_all.sh
./stop_all.sh
```

#### 2. 手动启动（用于调试）

1. 启动 `rosbridge_server`
```bash
source ~/raybot_core_ws/install/setup.bash
source /home/raybot/robotics_demos_py/install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  qos_overrides_path:=/home/raybot/ethercat_control_web/backend/config/qos_overrides.yaml \
  call_services_in_new_thread:=true \
  send_action_goals_in_new_thread:=true
```

2. 启动网页主后端 API 服务
```bash
cd ~/ethercat_control_web
source ~/raybot_core_ws/install/setup.bash
python3 backend/services/motor_control_server.py
```

3. 启动网页前端
```bash
cd ~/ethercat_control_web
npm run dev -- --host
```

**主后端启动标志**：
```
raybot@raybot:~/ethercat_control_web$ python3 backend/services/motor_control_server.py
🚀 网页api服务启动于 http://0.0.0.0:8889
   可以使用 http://<raybot-ip>:8889 从任意电脑连接
   已启用多线程支持并发请求
   ⚠️ 已使用 ROS2 publisher 方式（改进版）
   GET /api/motor-control/status          - 获取节点状态
   GET /api/motor-control/start           - 启动电机控制节点
   GET /api/motor-control/stop            - 停止电机控制节点
   GET /api/motor-control/refresh-motors  - 刷新电机列表
   POST /api/motor-control/config         - 发送 CSP 配置命令
```


### 开机默认启动网页服务
```bash
#查看实时运行日志
tail -f /home/raybot/ethercat_control_web/logs/startup.log
```
#### ⚙️ 开机自启管理

**禁用开机自启**
禁用crontab任务
```bash
crontab -e
```
删除或注释掉这行（前面加#）：

```bash
# @reboot sleep 15 && /home/raybot/ethercat_control_web/cron_start.sh
```
#### 临时停止服务
```bash
cd ~/ethercat_control_web
./stop_all.sh
```
#### 📁 日志文件位置
```
/home/raybot/ethercat_control_web/logs/startup.log
```
---

#### 2. 打开 Web 界面

>**⚠️注意**：在运行npm run dev 指令后，会根据板端ip自动生成网址如下（需对应局域网IP）
```bash
raybot@raybot:~/ethercat_control_web$ npm run dev -- --host

> ros2-webui@0.0.0 dev
> vite --host


  VITE v7.2.6  ready in 416 ms

  ➜  Local:   http://localhost:3210/
  ➜  Network: http://192.168.0.235:3210/ #浏览器输入该地址即可进入Ray_studio
  ➜  press h + enter to show help
```

#### 3. 验证网页端是否成功启动
>**⚠️注意**：浏览器设备需和开发板在同一局域网下，输入192.168.0.235:3210 若正常进入则成功

![浏览器输入ip](/docs/Ray_Studio快速入门指南/_resources/浏览器输入ip.png)

---

## 🖥️ Web 界面概览

### 界面布局

| 元素 | 功能 |
|------|------|
| **🏠 Ray Studio** | 顶部标题与品牌标识 |
| **📖 文档** | 打开独立文档中心页面 |
| **❓ 页面引导** | 显示当前页面的功能概述 |
| **中 / EN** | 一键切换中英文界面 |
| **☀️ / 🌙 / ⚪ 主题切换** | 循环切换亮色、深色、灰色主题 |
| **⚙️ 系统设置** | 打开系统配置面板 |

系统设置提供了丰富的功能配置选项，包括控制模式、界面外观、导航菜单显示等，用户可根据实际需求灵活定制
### 左侧导航菜单

点击左侧菜单项切换页面，所有菜单项都可在系统设置中独立显示/隐藏
| 菜单项 | 图标 | 功能 |
|--------|------|------|
| **电机控制** | ⚙️ | 电机参数监控和故障诊断 |
| **PID 调试** | 🔧 | 控制参数实时优化 |
| **工作日志** | 📝 | 系统操作历史和日志查看 |
| **Topic Terminal** | 📡 | ROS 2 消息和服务监控 |
| **系统诊断** | 🚑️ | 系统节点状态检查、健康诊断和强制关闭 |

### 左下角浮窗状态面板

<img src="/docs/Ray_Studio快速入门指南/_resources/delaypanel.png"
     alt="delaypanel"
     style="display: block; margin: 0;">

实时显示：
- **ROS 连接状态** 
- **当前延迟 ms**

---

## 主要功能模块详解

### 1. 电机控制
技术性文档请阅读[RAYBOT_EtherCAT_控制主站使用说明-V20.md](/documentation.html#ethercat-master)
**用途**：EtherCAT底层电机参数监控、故障诊断和精细控制。

#### 界面分布

**左侧：电机曲线显示** (Motor Viewer)
- 实时显示选中电机的：
  - 位置变化曲线
  - 速度变化曲线
  - 电流/力矩曲线

**右侧：电机控制面板** (Motor Control Panel)
- 工具栏
- 电机列表
- 电机动作编程器

#### 电机列表字段说明

| 字段 | 含义 |单位名称 |
|------|------|--------|
| **ID** | 电机编号 | 1-N |
| **Alias** | 电机别名 | 如 1100、1200 |
| **State** | EtherCAT 状态 | BUS_INIT、OPERATION_ENABLED 等 |
| **Fault** | 故障信息 | 来自 ROS2 `/motor_errors` topic |
| **Current(%)** | CST 目标力矩| 额定力矩的千分之一 |
| **Position(°)** | CSP 目标位置 | counts |
| **Velocity(°/s)** |CSV 目标速度 | °/s |

#### 故障排查

**悬停故障列显示详情**：
```
State: BUS_INIT
Fault_Active: true
AL Status Code: 0x8611
fault_messages: ["Over Current", "Position Limit"]
```


#### 电机控制方式

| 模式 | 说明 | 用途 |
|------|------|------|
| **位置模式** (Position) | 控制电机转到指定角度 | 精确位置控制 |
| **速度模式** (Velocity) | 控制电机以指定速度旋转 | 连续运动 |
| **力矩模式** (Torque) | 控制电机输出力矩 | 力控应用 |

#### 电机动作编程

**步骤**：
```
1.点击Refresh按键
2.选择电机、模式、目标值，可显示实时曲线
3.点击 "⚡️ Execute" 执行，点击 "⏹ Stop" 停止执行
4.循环点击色块可切换三种控制模式
5.在下方 "Motor Motion Programmer" 点击 "+ Add from List" 添加动作，可连续添加多个动作形成序列
6.点击 "▶️ Run" 执行勾选的序列

```

---

### 2. PID 调试

**用途**：实时修改电机控制参数，优化控制性能。

#### 界面分布

- **左侧**：PID 实时曲线
- **右侧**：PID 参数调试面板

#### PID 参数说明

| 参数 | 符号 | 说明 | 调参建议 |
|------|------|------|----------|
| **比例增益** | P | 对误差的直接响应 | 过大→振荡，过小→响应慢 |
| **积分增益** | I | 消除静差 | 过大→超调，过小→误差无法消除 |
| **微分增益** | D | 预测趋势，抑制超调 | 过大→噪声放大，过小→无阻尼效果 |


#### 调参步骤

```
1. 优先点击刷新，更新电机数量，选择目标电机（下拉菜单）
2. 先读取再输入新的 P、I、D 系数
3. 点击 "Write" 写入参数，掉电后恢复
4.进行速度阶跃
5. 观察左侧曲线变化
6. 继续微调直到满意
7. 点击 "Save" 永久写入该参数
```

---

### 3. 工作日志

> **⚠️ 提示**：该面板功能仅对电机页面事件进行优化，后续更新

**用途**：记录系统操作历史和事件。

#### 功能

- **自动记录**：系统启动、节点启动/停止、运动执行等事件
- **手动记录**：应用可调用 API 添加自定义日志
- **日志过滤**：按级别（信息/警告/错误）过滤

#### 日志级别

| 级别 | 颜色 | 用途 |
|------|------|------|
| **信息** (INFO) | 蓝色 | 一般操作信息 |
| **警告** (WARNING) | 橙色 | 需要注意的事项 |
| **错误** (ERROR) | 红色 | 系统故障和异常 |

#### 操作说明

```
1. 点击 "工作日志" 菜单进入
2. 右上角选择日志级别过滤
3. 可以看到时间戳、级别、消息三列
4. 点击 "清空日志" 清除所有记录
```

---

### 4. Topic Terminal

**用途**：实时监控和与 ROS 2 消息系统交互。

#### 功能特性

| 功能 | 说明 |
|------|------|
| **Topic 列表** | 自动发现所有 ROS 2 topics |
| **实时订阅** | 选择 topic 自动接收消息 |
| **消息查看** | 显示完整的消息内容和时间戳 |
| **命令执行** | 发送指定 topic 消息或调用服务 |
| **连接状态** | 显示与 ROSBridge 的连接状态 |

#### 界面说明

**左侧**：Topic 列表（共 N 个）
- 列出所有发现的 ROS2 topics
- 点击选择要查看的 topic

**右侧**：输出窗口
- 显示 topic 消息
- 每条消息显示时间戳和数据
- 自动向下滚动显示最新消息
- 顶部有清空和刷新按钮

**命令输入区**（正下方）：
```
$ top
  查看当前系统占用

$ ros2 topic list
  列出所有topic

$ kill pid
  清理进程
    
```

#### 常用 Topic 示例

```
# 查看电机错误信息
/motor_errors          → MotorErrorArray

# 查看电机状态
/motor_status          → MotorStatus

# 查看坐标变换
/tf                    → tf2_msgs/TFMessage
```

#### 连接故障排除

- **状态显示"Connecting"**：等待连接，通常需要 1-3 秒
- **状态显示"Disconnected"**：检查 ROSBridge 是否运行（`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`）
- **无法获取 Topics**：确保 ROS 系统正常运行，ROS Bridge 已启动

---
### 5. 系统诊断

**用途**：全面检查系统各组件的运行状态，并对异常服务执行启动、停止或强制关闭。

#### 检查项目

| 组件 | 检查内容 | 状态指示 |
|------|----------|----------|
| **⚙️ 电机控制节点** | API 状态 / 进程 / 运行时间 | 🟢 运行中 / 🔴 已停止 |
| **⚠️ 错误监测节点** | API 状态 / 进程状态 | 🟢 运行中 / 🔴 已停止 |
| **⏱️ 系统运行时间** | 主服务运行时长 / 最近更新时间 | 🟢 正常 |

#### 界面说明

每个服务卡片通常包含：
- 服务名称和状态徽章
- 当前状态详情
- 启动 / 停止 / 强制关闭按钮
- 相关指标（如 CPU、内存、运行时间、topic 数量）

#### 常见操作

**启动故障服务**：
```
1. 找到显示 "Stopped" 的服务卡片
2. 点击 "启动" 按钮
3. 等待 2-3 秒后再次刷新
4. 如仍失败，查看日志与 health 输出
```

**强制关闭服务**（仅在正常停止失败时使用）：
```
1. 点击 "强制杀死" 或 PID 强制关闭按钮
2. 进程会被立即终止
3. 再重新点击 "启动"
```

**自动刷新**：
```
勾选 "自动刷新" 后
系统会定时更新诊断状态
```

#### 诊断信息解读

- **最后更新**：最近一次刷新时间
- **系统运行时间**：自开机以来的运行时长

---

## ⚙️ 系统设置

点击顶部 **⚙️ 设置** 按钮进入系统配置。

### 设置分类及功能

#### 1️⃣ Display & Appearance（界面外观）

- **默认语言**：中文 / English
- **启用灰色主题**
- 顶部栏的语言按钮支持中英快速切换
- 顶部栏的主题按钮会循环切换亮色、深色、灰色主题

#### 2️⃣ Motor Personalization（电机个性化）

- **显示电机表格单位**
- **显示一键统一配置 Vmax**

#### 3️⃣ Navigation Editor（导航菜单编辑）

**用途**：自定义导航菜单显示/隐藏。

**操作步骤**：
```
1. 进入设置 → Navigation Editor
2. 看到 5 个菜单项，每个都有 Switch 开关：
   ✅ ON  = 菜单显示
   ❌ OFF = 菜单隐藏
3. 点击开关切换任意菜单项
4. 设置自动保存（无需手动点击）
5. 刷新页面配置仍然保留，多设备全同步
```

**菜单项对应关系**：
- 🟢 电机控制
- 🟢 PID 调试
- 🟢 工作日志
- 🟢 Topic Terminal
- 🟢 系统诊断

**特殊说明**：
- 若隐藏当前所在页面，刷新时自动切换到第一个可见菜单
- 顶部 **文档** 按钮不受导航菜单开关影响

#### 4️⃣ About（关于）

显示：
- 应用名称：Ray Studio
- 版本号
- 构建日期
- 许可证信息

---

## ❓ 常见问题解答

### Q1: 启动后网页显示空白或加载很慢？

**答**：

1. 检查后端是否运行：
```bash
ps aux | grep backend/services/motor_control_server.py
```

2. 如未运行，启动后端：
```bash
cd /home/raybot/ethercat_control_web
python3 backend/services/motor_control_server.py
```

3. 清除浏览器缓存：
```
浏览器 → 设置 → 清除缓存 → 刷新 (Ctrl+F5)
```

4. 尝试在隐私/无痕窗口打开

---

### Q2: ROS 显示"未连接"？

**答**：

**检查 1**：ROS 2 环境已配置
```bash
echo $ROS_DISTRO
# 应显示：humble
```

**检查 2**：ROS nodes 正常运行
```bash
ros2 node list
# 应看到多个节点
```

**检查 3**：ROSBridge 已启动（Topic Terminal 需要）
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**检查 4**：后端与 ROS 通信
- 查看后端日志是否有连接错误
- 重启后端


---

### Q3: 看到电机故障但不知道是什么原因？

**答**：

**悬停 Fault 列查看详情**：
```
State: BUS_INIT
Fault_Active: true  
AL Status Code: 0x8611
fault_messages: ["Over Current"]
```
---

### Q4: 隐藏菜单项后，刷新页面还是进入被隐藏的页面？

**答**：

已修复 ✅。系统现在会自动切换到第一个可见菜单。

如问题仍存在：
```
1. 清除浏览器缓存：Ctrl+Shift+Delete
2. 关闭所有标签页重新打开
3. 检查浏览器 Console 是否有错误（F12）
4. 重启后端：
   pkill -f backend/services/motor_control_server.py
   python3 backend/services/motor_control_server.py
```

---

### Q5: PID 调试页面无法显示曲线？

**答**：

1. **启动电机控制节点**：
   - 点击"Start Motor Control"按钮
   - 需耐心等待实时数据是否更新

2. **运动电机产生数据**：
   - 在电机控制页面运动目标电机
   - 曲线需要有数据才能显示

3. **选择要查看的电机**：
   - 在右侧下拉菜单选择电机
   - 如无可选项，检查电机列表是否为空

4. **刷新页面**：
   - Ctrl+R 刷新

---

### Q10: 如何重启后端并保留配置？

**答**：

所有配置（菜单显示、PID 参数等）自动保存到文件：
```
/home/raybot/ethercat_control_web/backend/config/system_settings.json
/home/raybot/ethercat_control_web/backend/config/navigation_settings.json
```

重启后端**不会**丢失配置：
```bash
# 停止后端
pkill -f backend/services/motor_control_server.py

# 启动后端
cd /home/raybot/ethercat_control_web
python3 backend/services/motor_control_server.py

# 刷新网页，配置恢复
```

---

## 🔗 快速链接

| 资源 | 链接 |
|------|------|
| 项目目录 | `/home/raybot/ethercat_control_web/` |
| 后端服务 | `http://localhost:8889` |
| Web 前端 | `http://localhost:3210` |
| 启动日志 | `/home/raybot/ethercat_control_web/logs/startup.log` |

---

## 📞 故障排除流程图

```
系统问题
    ↓
1. 是否能打开网页？ ── NO ──→ 检查后端进程
                            ↓
2. 是否显示"ROS已连接"？ ── NO ──→ 检查 ROS 环境
                                 ↓
3. 是否能看到电机列表？ ── NO ──→ 启动电机控制节点
                                 ↓
4. 是否有 Fault 标记？ ── YES ──→ 查看故障详情→排查
                      
5. 是否需要调参？ ────── YES ──→ 进入 PID 调试页面
```

---

**祝你使用愉快！🎉 有任何问题欢迎查阅本指南或查看系统诊断。**
