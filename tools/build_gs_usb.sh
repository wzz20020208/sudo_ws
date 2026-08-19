#!/usr/bin/env bash
# build_gs_usb.sh —— 用 RT 内核源码树补编 gs_usb 驱动模块并安装（不编全量内核）。
#
# 为什么必须用该源码树：模块 vermagic 必须与运行内核一致
#   （5.10.160-rt89+ SMP preempt_rt mod_unload aarch64）。
#   原装 rockchip-rk3588 内核编出的 gs_usb.ko vermagic 不同（含 modversions），
#   直接拷过来加载报 version magic mismatch —— 即「内核不匹配」。
#   源码树位置由 /lib/modules/<uname -r>/build 符号链接定位。
#
# 用法: bash tools/build_gs_usb.sh
# 前置: RT 源码树在 /root/linux-orangepi-5.10.160-rt（build 符号链接指向处）；
#       需要 root（内部自动 sudo，普通用户直接跑即可）。
# 产物: /lib/modules/<uname -r>/kernel/drivers/net/can/usb/gs_usb.ko + depmod -a。
# 详见 docs/00_环境与驱动.md。

set -euo pipefail

if (( EUID != 0 )); then
    SUDO="sudo"
else
    SUDO=""
fi

KVER="$(uname -r)"
BUILD_LINK="/lib/modules/${KVER}/build"

# 1. 定位 RT 源码树：用 readlink 读符号链接目标（不遍历 /root，避免权限问题）
if [[ -L "$BUILD_LINK" ]]; then
    KSRC="$(readlink "$BUILD_LINK")"
elif [[ -d "$BUILD_LINK" ]]; then
    KSRC="$BUILD_LINK"
else
    echo "ERROR: $BUILD_LINK 不存在，无法定位 RT 内核源码树。" >&2
    exit 1
fi
echo ">> 内核版本: $KVER"
echo ">> RT 源码树: $KSRC"
cd "$KSRC"

# 2. 确保 CONFIG_CAN_GS_USB=m（=y 编进内核不产 .ko；缺失则追加）
if ! $SUDO grep -q '^CONFIG_CAN_GS_USB=' .config; then
    echo ">> .config 无 CONFIG_CAN_GS_USB，追加 =m"
    echo 'CONFIG_CAN_GS_USB=m' | $SUDO tee -a .config >/dev/null
elif $SUDO grep -q '^CONFIG_CAN_GS_USB=y$' .config; then
    echo ">> .config 里 CONFIG_CAN_GS_USB=y，改为 =m（编模块才有 .ko）"
    $SUDO sed -i 's/^CONFIG_CAN_GS_USB=.*/CONFIG_CAN_GS_USB=m/' .config
fi
$SUDO grep '^CONFIG_CAN_GS_USB=' .config

# 3. 准备编译环境（生成 auto.conf、模块符号表等；已 build 过则幂等快速）
$SUDO make modules_prepare

# 4. 只编单个模块，不全量编内核
$SUDO make drivers/net/can/usb/gs_usb.ko

# 5. 安装到当前内核模块目录 + 刷新依赖
INSTALL_DIR="/lib/modules/${KVER}/kernel/drivers/net/can/usb"
$SUDO mkdir -p "$INSTALL_DIR"
$SUDO cp drivers/net/can/usb/gs_usb.ko "$INSTALL_DIR/"
$SUDO depmod -a

# 6. 验证 vermagic 与运行内核一致
echo ">> 安装完成，vermagic 核对："
$SUDO modinfo -F vermagic "$INSTALL_DIR/gs_usb.ko"
$SUDO modprobe gs_usb 2>/dev/null || true
echo ">> 完成。插上适配器后 ip link 应出现 can0（第二个适配器是 can1）。"
