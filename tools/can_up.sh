#!/usr/bin/env bash
# can_up.sh —— 一键启动 can0（1Mbps，接真实 CAN 总线）。
#
# 便捷入口：参数固定为电机联调默认值，省去每次敲 --ifname/--bitrate/--loopback。
# 完整选项（自定义接口名 / 环回自测）见 tools/can_setup.sh。
#
# 用法：
#   bash tools/can_up.sh          # 建 can0、1Mbps、非环回、up
#   bash tools/can_up.sh can1     # 指定其他接口名
#
# 需要 root 权限（内部自动 sudo）。

set -euo pipefail

IFNAME="${1:-can0}"

exec bash "$(dirname "$0")/can_setup.sh" --ifname "$IFNAME" --bitrate 1000000 --loopback off
