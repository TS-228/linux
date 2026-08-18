#!/usr/bin/env bash
# Exercise PIC fan/temp on mainline kernel (hwmon driver or uart_probe fallback).
#
# Usage:
#   tools/qnap-ts228/mainline-pic-test.sh [user@host] [pwm_value]
#
# Examples:
#   tools/qnap-ts228/mainline-pic-test.sh root@192.168.178.96
#   tools/qnap-ts228/mainline-pic-test.sh root@192.168.178.96 255

set -euo pipefail

HOST="${1:-root@192.168.178.96}"
PWM="${2:-}"
SSH_KEY="${SSH_KEY:-$HOME/.ssh/id_ed25519}"
SSH=(ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$HOST")

echo "=== kernel ==="
"${SSH[@]}" 'uname -r; zcat /proc/config.gz 2>/dev/null | grep QNAP_TS228 || true'

echo ""
echo "=== driver / serial ==="
"${SSH[@]}" 'lsmod | grep qnap_ts228 || echo "(module not loaded)"
ls -la /sys/bus/serial/drivers/qnap-ts228-pic/ 2>/dev/null || true
ls -la /dev/ttyS1 2>/dev/null || true'

H=$("${SSH[@]}" "grep -l qnapts228 /sys/class/hwmon/hwmon*/name 2>/dev/null | head -1 | xargs dirname" || true)

if [ -n "$H" ]; then
	echo ""
	echo "=== hwmon ($H) ==="
	"${SSH[@]}" "echo name: \$(cat $H/name)
for f in pwm1 fan1_input temp1_input; do
  if [ -f $H/\$f ]; then printf '%s: ' \$f; cat $H/\$f; fi
done
CDEV=\$(grep -l qnap-ts228 /sys/class/thermal/cooling_device*/type 2>/dev/null | head -1)
if [ -n \"\$CDEV\" ]; then
  echo cooling: \$(cat \$CDEV/type) cur_state=\$(cat \$CDEV/cur_state) max_state=\$(cat \$CDEV/max_state)
fi"

	if [ -n "$PWM" ]; then
		echo ""
		echo "=== set pwm1=$PWM ==="
		"${SSH[@]}" "test -f $H/pwm1 && echo $PWM > $H/pwm1; sleep 2; echo pwm1: \$(cat $H/pwm1); echo fan1_input: \$(cat $H/fan1_input 2>/dev/null || echo n/a)"
	fi
else
	echo ""
	echo "=== hwmon driver not found — try uart_probe ==="
	if "${SSH[@]}" 'test -x /tmp/uart_probe'; then
		"${SSH[@]}" '/tmp/uart_probe init; /tmp/uart_probe read-fan; /tmp/uart_probe read-temp' || true
	else
		echo "Build & deploy: make -C tools/qnap-ts228 deploy HOST=$HOST"
		echo "(requires ttyS1 free — stop/disable qnap-ts228-pic or ensure no other owner)"
	fi
fi

echo ""
echo "=== PIC EEPROM / power (serial0-0) ==="
"${SSH[@]}" 'DEV=/sys/bus/serial/devices/serial0-0
for f in serial_number mac_address power_recovery led_status led_usb; do
  if [ -f $DEV/\$f ]; then printf "%s: " \$f; cat $DEV/\$f; fi
done
echo eth0: $(cat /sys/class/net/eth0/address 2>/dev/null || echo n/a)'

echo ""
echo "=== recent dmesg (qnap/pic) ==="
"${SSH[@]}" 'dmesg | grep -iE qnap|pic | tail -15' || true
