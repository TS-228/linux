#!/bin/bash

CONFIG_FILE=".config"
CONFIG_BACKUP=".config.old"

# Check if config has changed and only clean if necessary
if [ -f "$CONFIG_FILE" ] && [ -f "$CONFIG_BACKUP" ] && ! cmp -s "$CONFIG_FILE" "$CONFIG_BACKUP"; then
    make ARCH=arm clean
fi

# Backup the current config
cp -f "$CONFIG_FILE" "$CONFIG_BACKUP"

# Configure the build
ARCH=arm make menuconfig

# Compile the kernel
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j$(nproc) zImage
ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- make dtbs

# Generate the uImage
#sudo dd if=/dev/zero bs=1 count=$((1024 - $(stat -c %s arch/arm/boot/zImage) % 1024)) >> arch/arm/boot/zImage

sudo mkimage -A arm -O linux -T kernel -a 0x02200000 -e 0x02200000 -C none -d arch/arm/boot/zImage /srv/tftp/emmc.uImage -p 4096
#sudo mkimage -A arm -O linux -T kernel -a 0x02200000 -e 0x02200000 -C none -d arch/arm/boot/zImage /srv/tftp/emmc.uImage
#mkimage -A arm -O linux -T kernel -a 0x02200000 -e 0x02200000 -C none -d arch/arm/boot/Image /srv/tftp/uImage -p 4096

# Copy the DTB file (if necessary)
#sudo cp arch/arm/boot/dts/realtek/rtd119x/rtd-119x-horseradish.dtb /srv/tftp/rescue.emmc.dtb
#sudo cp arch/arm/boot/dts/realtek/rtd119x/rtd-119x-qa-rescue.dtb /srv/tftp/rescue.emmc.dtb
sudo cp arch/arm/boot/dts/realtek/rtd119x/rtd-119x-ts228.dtb /srv/tftp/rescue.emmc.dtb

