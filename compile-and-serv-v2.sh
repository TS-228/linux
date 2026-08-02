#!/bin/bash
set -euo pipefail

# Arm GNU Toolchain 11.2 (recommended for Linux 5.15)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="${SCRIPT_DIR}/../compiler/gcc-arm-11.2-2022.02-x86_64-arm-none-linux-gnueabihf/bin:$PATH"
export CROSS_COMPILE=arm-none-linux-gnueabihf-
export ARCH=arm

# Older toolchains kept for reference:
#export PATH=/home/stephan/qnap-ts-228/compiler/gcc-linaro-4.8-2015.06-x86_64_arm-linux-gnueabi/bin:$PATH
#export PATH=/home/stephan/qnap-ts-228/compiler/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf/bin:$PATH
#export CROSS_COMPILE=arm-linux-gnueabihf-

CONFIG_FILE=".config"
CONFIG_BACKUP=".config.old"

make prepare

# Check if config has changed and only clean if necessary
if [ -f "$CONFIG_FILE" ] && [ -f "$CONFIG_BACKUP" ] && ! cmp -s "$CONFIG_FILE" "$CONFIG_BACKUP"; then
    echo "cleanup"
    make clean
fi

# Backup the current config
cp -f "$CONFIG_FILE" "$CONFIG_BACKUP"

# Configure the build (skip when non-interactive / SKIP_MENUCONFIG=1)
if [[ -z "${SKIP_MENUCONFIG:-}" && -t 0 ]]; then
    make menuconfig
else
    echo "Skipping menuconfig (non-interactive); running olddefconfig"
    make olddefconfig
fi

CONFIG_ADDR="0x00108000"

# Compile the kernel
make -j$(nproc) uImage LOADADDR=$CONFIG_ADDR
make dtbs

# Copy the DTB file (optional tftp deploy)
if [[ -z "${SKIP_TFTP_COPY:-}" ]]; then
    sudo cp arch/arm/boot/dts/realtek/rtd119x/rtd-119x-horseradish-QNAP-TS-X28.dtb /var/lib/tftpboot/rescue.emmc.dtb
    sudo cp arch/arm/boot/uImage /var/lib/tftpboot/emmc.uImage
fi

echo "tftp \$fdt_loadaddr \$serverip:\$rescue_dtb && tftp \$kernel_loadaddr \$serverip:\$rescue_vmlinux && bootm \$kernel_loadaddr - \$fdt_loadaddr"
