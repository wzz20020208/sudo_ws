#!/usr/bin/env bash
# can_setup.sh —— 配置并启动 SocketCAN 接口。
#
# 针对 gs_usb / canable 这类 USB-CAN 适配器：接口由内核驱动在设备插上时自动创建，
# 因此不能 `ip link add`（已存在 → 报 File exists），也不能 `ip link del`
# （会把硬件接口删掉）。正确做法：接口应在；只 `ip link set ... type can bitrate`
# 改参数，再 up。
#
# 用法：
#   bash tools/can_setup.sh --ifname can0 --bitrate 1000000 --loopback off
#
#   --loopback on  接口进入环回模式（无硬件时自测收发通路）
#   --loopback off 正常接入真实 CAN 总线（RH 关节模组要求 1Mbps）
#
# 需要 root 权限（内部自动 sudo）。

set -euo pipefail

IFNAME="can0"
BITRATE=1000000
LOOPBACK=off

if (( EUID != 0 )); then
    SUDO="sudo"
else
    SUDO=""
fi

usage() {
    cat <<EOF
Usage: $0 [options]
  --ifname <if>        CAN interface name (default: can0)
  --bitrate <bps>      bitrate in bit/s (default: 1000000)
  --loopback <on|off>  interface loopback mode: on=环回自测 off=接真实总线 (default: off)
  -h, --help           show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ifname)   IFNAME="$2";   shift 2 ;;
        --bitrate)  BITRATE="$2";  shift 2 ;;
        --loopback) LOOPBACK="$2"; shift 2 ;;
        -h|--help)  usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

# 硬件接口（gs_usb 等）插上即自动创建；缺失说明适配器没插或驱动未加载
if ! ip link show "$IFNAME" >/dev/null 2>&1; then
    echo "ERROR: 接口 $IFNAME 不存在。USB-CAN 适配器应插上即自动创建；请检查是否插入/驱动加载。" >&2
    exit 1
fi

echo ">> taking $IFNAME down (if up)"
$SUDO ip link set "$IFNAME" down 2>/dev/null || true

echo ">> configuring $IFNAME type can bitrate=$BITRATE loopback=$LOOPBACK"
$SUDO ip link set "$IFNAME" type can bitrate "$BITRATE" loopback "$LOOPBACK"

echo ">> bringing $IFNAME up"
$SUDO ip link set "$IFNAME" up

echo ">> done:"
ip -details link show "$IFNAME"
