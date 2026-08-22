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
CONFIG_ARM_SNAPSHOT=".config.arm"
CONFIG_HISTORY_DIR=".config.d"
TFTP_UIMAGE="${TFTP_UIMAGE:-/var/lib/tftpboot/emmc.uImage}"

is_arm_rtd119x_config() {
	local f=$1
	[[ -f "$f" ]] || return 1
	grep -q '^# Linux/arm' "$f" || return 1
	grep -q '^CONFIG_ARCH_REALTEK=y' "$f" || return 1
	return 0
}

backup_config() {
	local src=$1
	mkdir -p "$CONFIG_HISTORY_DIR"
	cp -f "$src" "$CONFIG_HISTORY_DIR/config-$(date +%Y%m%d-%H%M%S)"
	cp -f "$src" "$CONFIG_BACKUP"
}

try_restore_from_uimage() {
	local uimage=$1

	[[ -f "$uimage" ]] || return 1
	scripts/extract-ikconfig "$uimage" > "$CONFIG_FILE" 2>/dev/null || return 1
	is_arm_rtd119x_config "$CONFIG_FILE" || return 1
	echo "Restored ARM config from $uimage (IKCONFIG embedded)"
	return 0
}

ensure_arm_config() {
	local hdr

	if is_arm_rtd119x_config "$CONFIG_FILE"; then
		return
	fi

	hdr=$(grep -m1 '^# Linux/' "$CONFIG_FILE" 2>/dev/null || echo "(missing)")

	if [[ -f "$CONFIG_ARM_SNAPSHOT" ]] && is_arm_rtd119x_config "$CONFIG_ARM_SNAPSHOT"; then
		echo "Restoring last good ARM config from $CONFIG_ARM_SNAPSHOT (was: $hdr)"
		cp -f "$CONFIG_ARM_SNAPSHOT" "$CONFIG_FILE"
		return
	fi

	if [[ -f "$CONFIG_BACKUP" ]] && is_arm_rtd119x_config "$CONFIG_BACKUP"; then
		echo "Restoring ARM config from $CONFIG_BACKUP (was: $hdr)"
		cp -f "$CONFIG_BACKUP" "$CONFIG_FILE"
		return
	fi

	if [[ -f .config.working-from-uImage ]] && is_arm_rtd119x_config .config.working-from-uImage; then
		echo "Restoring ARM config from .config.working-from-uImage (was: $hdr)"
		cp -f .config.working-from-uImage "$CONFIG_FILE"
		cp -f "$CONFIG_FILE" "$CONFIG_ARM_SNAPSHOT"
		return
	fi

	if try_restore_from_uimage "$TFTP_UIMAGE"; then
		cp -f "$CONFIG_FILE" "$CONFIG_ARM_SNAPSHOT"
		return
	fi

	if try_restore_from_uimage "arch/arm/boot/uImage"; then
		cp -f "$CONFIG_FILE" "$CONFIG_ARM_SNAPSHOT"
		return
	fi

	if [[ ! -f "$CONFIG_FILE" ]]; then
		echo "No .config — creating ARM RTD119x baseline"
	else
		echo "WARNING: .config is not a valid RTD119x ARM config ($hdr)"
		echo "Reinitializing: multi_v7_defconfig + ARCH_REALTEK"
	fi
	make multi_v7_defconfig
	./scripts/config --enable ARCH_REALTEK
	./scripts/config --enable COMMON_CLK_RTD119X
	./scripts/config --disable COMMON_CLK_REALTEK_DEBUG
	make olddefconfig
	cp -f "$CONFIG_FILE" "$CONFIG_ARM_SNAPSHOT"
}

apply_emmc_config() {
	# Previously kept as a module: built-in (=y) used to break early boot with
	# the old gzip-wrapped raw Image (~21 KiB larger image hit an
	# undefined-instruction at 0x0010800c). Now built-in for testing since the
	# boot path has since moved to zImage; revert to --module if that recurs.
	./scripts/config --enable MMC_RTK_EMMC
	./scripts/config --disable IKCONFIG_PROC
	make olddefconfig
}

apply_boot_config() {
	# Match linux-4.9.119 RTD119x: fixed PHYS_OFFSET, no runtime ZRELADDR.
	./scripts/config --disable AUTO_ZRELADDR
	./scripts/config --set-val PHYS_OFFSET 0x0
	make olddefconfig
}

apply_usb_phy_config() {
	# DWC3 uses upstream-style PHY drivers in drivers/phy/realtek/
	./scripts/config --enable PHY_RTK_RTD_USB2PHY
	./scripts/config --enable PHY_RTK_RTD_USB3PHY
	# EHCI/OHCI still use the vendor RLE0599 PHY driver.
	./scripts/config --enable RTK_USB_RLE0599_PHY
	make olddefconfig
}

apply_qnap_ts228_pic_config() {
	./scripts/config --enable SERIAL_DEV_BUS
	./scripts/config --enable SERIAL_8250
	./scripts/config --enable SERIAL_8250_DW
	./scripts/config --enable HWMON
	./scripts/config --enable THERMAL
	./scripts/config --enable SENSORS_QNAP_TS228_PIC
	./scripts/config --disable MFD_QNAP_MCU
	./scripts/config --disable SENSORS_QNAP_MCU_HWMON
	./scripts/config --disable CONFIG_INPUT_QNAP_MCU 2>/dev/null || \
		./scripts/config --disable INPUT_QNAP_MCU 2>/dev/null || true
	./scripts/config --disable LEDS_QNAP_MCU 2>/dev/null || true
	make olddefconfig
}

apply_ksmbd_config() {
	# Kernel SMB3 server (ksmbd) built-in.
	./scripts/config --enable NETWORK_FILESYSTEMS
	./scripts/config --enable SMB_SERVER
	./scripts/config --enable SMB_SERVER_CHECK_CAP_NET_ADMIN
	./scripts/config --disable SMB_SERVER_KERBEROS5
	./scripts/config --disable SMB_SERVER_SMBDIRECT
	make olddefconfig
}

apply_cpufreq_config() {
	# Dynamic scaling: all OPPs work via cpufreq-dt; performance default
	# keeps the CPU stuck at 800 MHz.
	./scripts/config --enable CPU_FREQ
	./scripts/config --enable CPUFREQ_DT
	./scripts/config --enable CPU_FREQ_GOV_SCHEDUTIL
	./scripts/config --enable CPU_FREQ_GOV_ONDEMAND
	./scripts/config --enable CPU_FREQ_GOV_PERFORMANCE
	./scripts/config --disable CPU_FREQ_DEFAULT_GOV_PERFORMANCE
	./scripts/config --enable CPU_FREQ_DEFAULT_GOV_SCHEDUTIL
	make olddefconfig
}

apply_thermal_config() {
	./scripts/config --enable THERMAL
	./scripts/config --enable THERMAL_OF
	./scripts/config --enable RTK_THERMAL
	./scripts/config --enable RTK_THERMAL_RTD119X
	./scripts/config --enable RTK_EFUSE
	./scripts/config --enable NVMEM
	./scripts/config --enable NVMEM_SYSFS
	./scripts/config --enable SENSORS_QNAP_TS228_PIC
	# Vendor Android stub — not a real battery on TS-228.
	./scripts/config --disable RTD1XXX_POWER 2>/dev/null || true
	make olddefconfig
}

apply_armv7_neon_config() {
	# RTD1195 = dual Cortex-A7 with NEON, no ARMv8 Crypto Extensions.
	# Enable NEON-accelerated crypto; skip *ARM_CE* (needs PMULL/AES instructions).
	./scripts/config --enable CRYPTO_SHA1_ARM
	./scripts/config --enable CRYPTO_SHA1_ARM_NEON
	./scripts/config --enable CRYPTO_SHA256_ARM
	./scripts/config --enable CRYPTO_SHA512_ARM
	./scripts/config --enable CRYPTO_AES_ARM
	./scripts/config --enable CRYPTO_AES_ARM_BS
	./scripts/config --enable CRYPTO_BLAKE2S_ARM
	./scripts/config --enable CRYPTO_BLAKE2B_NEON
	./scripts/config --enable CRYPTO_NHPOLY1305
	./scripts/config --enable CRYPTO_NHPOLY1305_NEON
	./scripts/config --enable CRYPTO_GHASH_ARM_CE
	# Broken on this board — use CPU NEON crypto instead.
	./scripts/config --disable CRYPTO_DEV_RTK_MCP
	./scripts/config --disable CRYPTO_DEV_RTK_MCP_SHA_COMPLIANCE_TEST
	make olddefconfig
}

ensure_arm_config

apply_emmc_config
apply_boot_config
apply_usb_phy_config
apply_qnap_ts228_pic_config
apply_ksmbd_config
apply_cpufreq_config
apply_thermal_config
apply_armv7_neon_config

if ! is_arm_rtd119x_config "$CONFIG_FILE"; then
	echo "ERROR: .config is still not a valid RTD119x ARM config after setup" >&2
	exit 1
fi

make prepare

# Check if config has changed and only clean if necessary
if [ -f "$CONFIG_FILE" ] && [ -f "$CONFIG_BACKUP" ] && ! cmp -s "$CONFIG_FILE" "$CONFIG_BACKUP"; then
	echo "cleanup"
	make clean
fi

# Backup the current config (keep timestamped history)
backup_config "$CONFIG_FILE"

# Only open menuconfig when explicitly requested (avoids surprise prompts)
if [[ -n "${FORCE_MENUCONFIG:-}" && -t 0 ]]; then
	make menuconfig
	apply_emmc_config
	apply_boot_config
elif [[ -z "${SKIP_MENUCONFIG:-}" && -t 0 && ! -f "$CONFIG_ARM_SNAPSHOT" ]]; then
	make menuconfig
	apply_emmc_config
	apply_boot_config
else
	echo "Using saved .config (set FORCE_MENUCONFIG=1 to edit interactively)"
fi

# Keep a known-good ARM snapshot after successful configure
cp -f "$CONFIG_FILE" "$CONFIG_ARM_SNAPSHOT"

CONFIG_ADDR="0x00108000"

# U-Boot gunzip is capped at CONFIG_SYS_BOOTM_LEN (8 MiB on RTD119x). A gzip-wrapped
# raw Image is ~16 MiB and hangs at "Uncompressing Kernel Image ...".
# Use zImage instead and set the uImage entry to the branch target after the header
# (Realtek U-Boot XIP can execute the 0x016f2818 magic if ep=load).
zimage_entry_addr() {
	python3 - <<'PY'
import struct, sys
load = int("0x00108000", 16)
with open("arch/arm/boot/zImage", "rb") as f:
    d = f.read(0x40)
branch, = struct.unpack("<I", d[0x20:0x24])
if branch >> 24 != 0xEA:
    sys.exit("zImage: no ARM branch at 0x20")
imm = branch & 0xFFFFFF
if imm & 0x800000:
    imm -= 0x1000000
print(hex(load + 0x20 + 8 + (imm << 2)))
PY
}

build_uimage() {
	local entry

	echo "Building zImage uImage (entry past zImage header, not gzip Image)"
	make -j"$(nproc)" zImage LOADADDR="$CONFIG_ADDR"
	entry=$(zimage_entry_addr)
	echo "uImage load=$CONFIG_ADDR entry=$entry"
	make uImage LOADADDR="$CONFIG_ADDR" UIMAGE_ENTRYADDR="$entry"
	file arch/arm/boot/uImage
}

build_uimage
make -j$(nproc) modules
make dtbs

# Always deploy boot artifacts + loadable modules to TFTP
sudo cp arch/arm/boot/dts/realtek/rtd1195-qnap-ts-228.dtb /var/lib/tftpboot/rescue.emmc.dtb
sudo cp arch/arm/boot/uImage /var/lib/tftpboot/emmc.uImage
if [[ -f drivers/mmc/host/rtkemmc_rtd119x.ko ]]; then
	sudo cp drivers/mmc/host/rtkemmc_rtd119x.ko /var/lib/tftpboot/
fi

KREL=$(make -s kernelrelease)
echo ""
echo "After TFTP boot, load eMMC module on the device:"
echo "  scp /var/lib/tftpboot/rtkemmc_rtd119x.ko root@<device>:/tmp/"
echo "  insmod /tmp/rtkemmc_rtd119x.ko"
echo "  lsblk   # expect mmcblk1 (mmc0 = SD slot)"
echo ""
echo "Permanent install (on USB rootfs):"
echo "  install -D -m 644 drivers/mmc/host/rtkemmc_rtd119x.ko \\"
echo "    /lib/modules/${KREL}/kernel/drivers/mmc/host/rtkemmc_rtd119x.ko"
echo "  echo rtkemmc_rtd119x > /etc/modules-load.d/rtkemmc.conf"
echo ""
echo "PIC driver (built-in):"
echo "  tools/qnap-ts228/mainline-pic-test.sh [host] [pwm]"
echo "  make -C tools/qnap-ts228 uart_probe && make -C tools/qnap-ts228 deploy HOST=root@<host>"
echo "  tools/qnap-ts228/pull-stock.sh admin@<stock-host>   # refresh offline HAL reference"
echo ""
echo "Rootfs cleanup (remove stale out-of-tree modules after PIC built-in):"
echo "  rm -f /lib/modules/${KREL}/extra/qnap-ts228-pic.ko"
echo "  rm -f /lib/modules/${KREL}/kernel/drivers/hwmon/qnap-ts228-pic.ko"
echo "  rm -f /etc/modules-load.d/qnap-ts228-pic.conf"
echo "  depmod -a"

echo "tftp \$fdt_loadaddr \$serverip:\$rescue_dtb && tftp \$kernel_loadaddr \$serverip:\$rescue_vmlinux && bootm \$kernel_loadaddr - \$fdt_loadaddr"
