#!/usr/bin/env bash
# build_vcan.sh —— 编译并加载 vcan（虚拟 CAN）模块，用于无硬件自测。
#
# 背景：当前内核未编译 vcan（CONFIG_CAN_VCAN 未开启），所以要先用内核源码树
# 编译出 vcan.ko 再加载。vcan 模块极小且稳定，加载后即可 `ip link add vcan0`。
#
# 用法：sudo bash tools/build_vcan.sh

set -euo pipefail

KREL="$(uname -r)"
KSRC="/root/linux-orangepi-5.10.160-rt"     # 内核源码树（build 符号链接指向它）
WORK="$(mktemp -d /tmp/vcanbuild.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

if (( EUID != 0 )); then
    echo "错误：本脚本需要 root，请用：sudo bash $0" >&2
    exit 1
fi

echo ">> 1/4 获取 vcan.c 源码"
if [ -f "$KSRC/drivers/net/can/vcan.c" ]; then
    cp "$KSRC/drivers/net/can/vcan.c" "$WORK/vcan.c"
    echo "    来自内核源码树: $KSRC"
else
    echo "    源码树中无 vcan.c，从 GitHub 拉取 linux v5.10 的 vcan.c"
    curl -fsSL "https://raw.githubusercontent.com/torvalds/linux/v5.10/drivers/net/can/vcan.c" \
        -o "$WORK/vcan.c"
fi
echo "    -> $(wc -l < "$WORK/vcan.c") 行"

echo ">> 2/4 生成 Makefile"
cat > "$WORK/Makefile" <<'EOF'
obj-m += vcan.o
EOF

echo ">> 3/4 对内核 ${KREL} 编译 vcan.ko"
make -C /lib/modules/"${KREL}"/build M="$WORK" modules

echo ">> 4/4 加载 vcan.ko 并创建 vcan0"
insmod "$WORK/vcan.ko" && echo "    vcan.ko 已加载"
ip link add dev vcan0 type vcan
ip link set vcan0 up
echo "    完成，当前 vcan 接口："
ip -details link show vcan0
