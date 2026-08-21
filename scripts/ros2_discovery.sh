#!/bin/bash
# ros2_discovery.sh — ROS 2 FastDDS 发现服务器管理脚本（服务器端跑在本机）
#
# 背景: 本网络（WiFi）丢弃客户端间组播，DDS 默认组播发现不可用；改用
#       Discovery Server 单播发现绕开（服务器跑在本机 172.16.53.157:11811）。
#
# 用法: ./ros2_discovery.sh start|stop|restart|status|env
#   start    启动发现服务器（已在跑则跳过，幂等）
#   stop     停止服务器
#   restart  重启
#   status   查看进程/端口状态
#   env      打印两台板每个终端需要执行的 export（客户端配置说明）
#
# 注意: 服务器 IP 是 DHCP 的，若本机 IP 变化，改下面的 SERVER_IP 并重启。

set -u

SERVER_IP="172.16.53.157"   # 本机（服务器）IP，两台板的 ROS_DISCOVERY_SERVER 都指向它
SERVER_PORT=11811
LOG=/tmp/fastdds_server.log

# 本机 IP 变化时可用 --ip 覆盖（如 ./ros2_discovery.sh --ip 172.16.53.200 start）
if [ "${1:-}" = "--ip" ] && [ $# -ge 3 ]; then
    SERVER_IP="$2"; shift 2
fi
CMD="${1:-help}"

server_pid() {
    pgrep -f "fast-discovery-server -i 0 -p ${SERVER_PORT}" | head -1
}

do_start() {
    set +u   # ROS setup.bash 内部有未定义变量，与 set -u 不兼容
    source /opt/ros/humble/setup.bash
    set -u
    if [ -n "$(server_pid)" ]; then
        echo "发现服务器已在运行 (pid $(server_pid))，无需重复启动"
        return 0
    fi
    nohup fastdds discovery -i 0 -p "$SERVER_PORT" > "$LOG" 2>&1 < /dev/null &
    sleep 1
    if [ -n "$(server_pid)" ]; then
        echo "发现服务器已启动 (pid $(server_pid)，端口 $SERVER_PORT，日志 $LOG)"
    else
        echo "启动失败，日志尾部：" >&2
        tail -5 "$LOG" >&2
        return 1
    fi
}

do_stop() {
    local pid; pid=$(server_pid)
    if [ -z "$pid" ]; then
        echo "发现服务器未在运行"
        return 0
    fi
    kill "$pid"
    echo "已停止 (pid $pid)"
}

do_status() {
    local pid; pid=$(server_pid)
    if [ -n "$pid" ]; then
        echo "运行中: pid $pid"
        ss -ulnp 2>/dev/null | grep ":${SERVER_PORT}" || true
    else
        echo "未运行"
    fi
}

do_env() {
    cat <<EOF
== 客户端（两台板）配置 ==
每个终端执行（新开终端必须重新执行）:
  source /opt/ros/humble/setup.bash
  export ROS_DISCOVERY_SERVER=${SERVER_IP}:${SERVER_PORT}

或写入 ~/.bashrc 永久生效（两台板都执行）:
  echo 'export ROS_DISCOVERY_SERVER=${SERVER_IP}:${SERVER_PORT}' >> ~/.bashrc

验证: A 机 ros2 topic pub --rate 5 /net_test std_msgs/msg/String "data: hi"
      B 机 ros2 topic echo /net_test
EOF
}

case "$CMD" in
    start)   do_start ;;
    stop)    do_stop ;;
    restart) do_stop; sleep 1; do_start ;;
    status)  do_status ;;
    env)     do_env ;;
    *) echo "用法: $0 [--ip <本机IP>] start|stop|restart|status|env" ;;
esac
