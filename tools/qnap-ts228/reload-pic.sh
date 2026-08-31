#!/usr/bin/env bash
# Build qnap-ts228-pic.ko and hot-reload on the NAS (no kernel reboot).
#
# Requires a kernel booted WITH modular PIC support (CONFIG_SENSORS_QNAP_TS228_PIC=m).
# One TFTP/kernel reboot is needed to switch away from built-in (=y); after that, use
# this script for driver iteration.
#
# Usage:
#   tools/qnap-ts228/reload-pic.sh [user@host]
#
# Defaults: root@192.168.178.96, SSH key ~/.ssh/id_ed25519

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HOST="${1:-root@192.168.178.96}"
SSH_KEY="${SSH_KEY:-$HOME/.ssh/id_ed25519}"
SSH=(ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no)
SCP=(scp -i "$SSH_KEY" -o StrictHostKeyChecking=no)
KO="$ROOT/drivers/hwmon/qnap-ts228-pic.ko"
REMOTE=/tmp/qnap-ts228-pic.ko
SERDEV=serial0-0
DRIVER=qnap-ts228-pic

cd "$ROOT"

export PATH="${PATH:-}:${ROOT}/../compiler/gcc-arm-11.2-2022.02-x86_64-arm-none-linux-gnueabihf/bin"
export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabihf-

if ! grep -q '^CONFIG_SENSORS_QNAP_TS228_PIC=m' .config 2>/dev/null; then
	echo "ERROR: CONFIG_SENSORS_QNAP_TS228_PIC is not =m in .config." >&2
	echo "Run: ./scripts/config --module SENSORS_QNAP_TS228_PIC && make olddefconfig" >&2
	echo "Then TFTP-boot the rebuilt kernel once; after that this script avoids reboots." >&2
	exit 1
fi

# .config can say =m while include/config/auto.conf is stale (=y) until syncconfig.
if grep -q '^CONFIG_SENSORS_QNAP_TS228_PIC=y' include/config/auto.conf 2>/dev/null; then
	echo "Syncing Kconfig (auto.conf was stale) ..."
	make syncconfig
fi

echo "Building drivers/hwmon/qnap-ts228-pic.ko ..."
make M=drivers/hwmon modules

echo "Installing on $HOST ..."
"${SCP[@]}" "$KO" "$HOST:$REMOTE"

"${SSH[@]}" "$HOST" "set -e
cfg=\$(zcat /proc/config.gz 2>/dev/null || true)
if [ -n \"\$cfg\" ]; then
	if ! echo \"\$cfg\" | grep -q '^CONFIG_SENSORS_QNAP_TS228_PIC=m'; then
		echo 'ERROR: running kernel lacks CONFIG_SENSORS_QNAP_TS228_PIC=m.' >&2
		echo 'TFTP-boot compile-and-serv-v2.sh kernel once, then re-run.' >&2
		exit 1
	fi
elif [ -e /sys/bus/serial/drivers/$DRIVER/$SERDEV ] && ! lsmod | grep -q '^qnap_ts228_pic '; then
	echo 'ERROR: PIC driver is built into the kernel (not a module).' >&2
	echo 'TFTP-boot a modular build once, then re-run.' >&2
	exit 1
fi
if [ -e /sys/bus/serial/drivers/$DRIVER/$SERDEV ]; then
	echo unbind $SERDEV
	echo $SERDEV > /sys/bus/serial/drivers/$DRIVER/unbind
fi
if lsmod | grep -q '^qnap_ts228_pic '; then
	echo rmmod qnap_ts228_pic
	rmmod qnap_ts228_pic
fi
echo insmod $REMOTE
insmod $REMOTE
if [ ! -e /sys/bus/serial/drivers/$DRIVER/$SERDEV ]; then
	echo bind $SERDEV
	echo $SERDEV > /sys/bus/serial/drivers/$DRIVER/bind
fi
sleep 1
dmesg | tail -8 | grep -i qnap || dmesg | tail -5
H=\$(grep -l qnapts228 /sys/class/hwmon/hwmon*/name 2>/dev/null | head -1 | xargs dirname)
if [ -n \"\$H\" ]; then
	echo hwmon: \$H
	cat \$H/name
fi
DEV=/sys/bus/serial/devices/$SERDEV
if [ -e \$DEV/led_status ]; then
	echo leds: \$DEV/led_{status,usb}  '(status: 0=off 1=green 2=red 3=blink; usb: 0=off 1=on 2=blink)'
fi
if [ -e \$DEV/serial_number ]; then
	echo -n 'serial_number: '; cat \$DEV/serial_number
	echo -n 'mac_address: '; cat \$DEV/mac_address
	echo -n 'power_recovery: '; cat \$DEV/power_recovery
fi
if [ -e \$DEV/buzzer ]; then
	echo buzzer: \$DEV/buzzer '(write: short|long|0|1)'
fi
"

echo "Done. Test: cat /sys/class/hwmon/hwmon*/fan1_input"
