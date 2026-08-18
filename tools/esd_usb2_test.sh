#!/usr/bin/env bash
# esd_usb2_test.sh —— 用打补丁的 esd_usb2 内核驱动尝试驱动 ECHO USBCAN-UC12 (0471:1200)
#
# 原理：板子丝印 "esd can-1-v1.0"，疑似 esd CAN-USB/2 克隆。若固件协议兼容，
# 把 0471:1200 加进 esd_usb2 设备表后，插上即可出 can0（SocketCAN），无需任何
# 厂商库或刷固件。本脚本：取源码 -> 注入 VID -> 编译 -> 加载 -> 检测。
#
# 用法：sudo bash tools/esd_usb2_test.sh

set -euo pipefail

KREL="$(uname -r)"
KSRC="/root/linux-orangepi-5.10.160-rt"       # 本机内核源码树
SRC_REL="drivers/net/can/usb/esd_usb2.c"
WORK="$(mktemp -d /tmp/esdtest.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

if (( EUID != 0 )); then
    echo "错误：需要 root，请用：sudo bash $0" >&2
    exit 1
fi

echo ">> 0/6 从内核源码树获取 esd_usb2.c"
if [ ! -f "$KSRC/$SRC_REL" ]; then
    echo "    源码树中不存在 $KSRC/$SRC_REL" >&2
    exit 1
fi
cp "$KSRC/$SRC_REL" "$WORK/esd_usb2.c"
echo "    OK ($(wc -l < "$WORK/esd_usb2.c") 行)"

echo ">> 1/6 当前设备表（补丁前）"
grep -n -A6 "usb_device_id esd_usb2_table" "$WORK/esd_usb2.c" | head -12

echo ">> 2/6 注入 0471:1200 设备项"
if grep -q "0x0471" "$WORK/esd_usb2.c"; then
    echo "    已存在 0x0471，跳过注入"
else
    sed -i '/^[[:space:]]*{ *USB_DEVICE(USB_ESDGMBH_VENDOR_ID, USB_CANUSBM_PRODUCT_ID)/a\	{USB_DEVICE(0x0471, 0x1200)},  /* ECHO USBCAN-UC12 */' \
        "$WORK/esd_usb2.c"
    if grep -q "0x0471" "$WORK/esd_usb2.c"; then
        echo "    注入成功："
        grep -n -A8 "usb_device_id esd_usb2_table" "$WORK/esd_usb2.c" | head -14
    else
        echo "    注入失败（未匹配到设备表行），请检查源码" >&2
        exit 1
    fi
fi

echo ">> 3/6 检查 probe 中是否有硬性 PID 门限"
grep -n "PRODUCT_ID\|version <\|VERSION" "$WORK/esd_usb2.c" | head -12 || true

echo ">> 4/6 编译 esd_usb2.ko"
cat > "$WORK/Makefile" <<'EOF'
obj-m += esd_usb2.o
EOF
make -C /lib/modules/"${KREL}"/build M="$WORK" modules

echo ">> 5/6 加载模块（若已加载先卸载）"
rmmod esd_usb2 2>/dev/null || true
insmod "$WORK/esd_usb2.ko" || { echo "    insmod 失败，最近内核日志："; dmesg | tail -20; exit 1; }
sleep 2   # 等待设备 probe
echo "    加载完成，驱动绑定情况："
lsusb | grep -i 0471 || true
if [ -d "/sys/bus/usb/drivers/esd_usb2" ]; then
    ls /sys/bus/usb/drivers/esd_usb2/ 2>/dev/null | grep -v ":" || echo "    (驱动已加载但未绑定任何设备)"
fi

echo ">> 6/6 检测 can 接口"
if ip -o link show 2>/dev/null | grep -qE "can[0-9]"; then
    echo "    🎉 出现 CAN 接口："
    ip -details link show | grep -B1 -A3 "can[0-9]"
    echo "    配置为 1Mbps 并启用："
    for c in $(ip -o link show | grep -oE "can[0-9]+"); do
        ip link set "$c" type can bitrate 1000000
        ip link set "$c" up
        echo "    -> $c UP (1Mbps)"
    done
else
    echo "    ❌ 未出现 canX 接口 —— 固件可能不是 esd 协议，见上方 dmesg"
fi

echo "--- 最近内核日志 ---"
dmesg | tail -15
