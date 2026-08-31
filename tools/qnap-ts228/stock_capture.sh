#!/bin/bash
# Run on stock QTS (TS-228) as admin to collect PIC/fan HAL context.
# Usage: sh stock_capture.sh [output_dir]

set -e
OUT="${1:-/tmp/qnap-pic-capture}"
mkdir -p "$OUT"

echo "=== model / HAL config ===" | tee "$OUT/summary.txt"
grep -E 'PIC|FAN|MCU|UART|ttyS' /etc/model.conf 2>/dev/null | tee -a "$OUT/summary.txt" || true
grep -E 'PIC|FAN|MCU' /etc/default/hal_daemon.cfg 2>/dev/null | tee -a "$OUT/summary.txt" || true

echo | tee -a "$OUT/summary.txt"
echo "=== processes ===" | tee -a "$OUT/summary.txt"
ps w | grep -E 'hal|pic|fan' | grep -v grep | tee -a "$OUT/summary.txt" || true

echo | tee -a "$OUT/summary.txt"
echo "=== serial devices ===" | tee -a "$OUT/summary.txt"
ls -la /dev/ttyS* 2>&1 | tee -a "$OUT/summary.txt"

echo | tee -a "$OUT/summary.txt"
echo "=== hal get fan (if hal_get_fan_speed exists) ===" | tee -a "$OUT/summary.txt"
for cmd in /sbin/hal_get_fan_speed /usr/sbin/hal_get_fan_speed \
	   /sbin/get_fan_speed /usr/local/sbin/hal_get_fan_speed; do
	if [ -x "$cmd" ]; then
		echo "# $cmd" | tee -a "$OUT/summary.txt"
		"$cmd" 2>&1 | tee -a "$OUT/summary.txt" || true
	fi
done

echo | tee -a "$OUT/summary.txt"
echo "=== lsof ttyS1 ===" | tee -a "$OUT/summary.txt"
lsof /dev/ttyS1 2>&1 | tee -a "$OUT/summary.txt" || true

echo | tee -a "$OUT/summary.txt"
echo "=== strings PIC/FAN from hal lib ===" | tee -a "$OUT/summary.txt"
for lib in /lib/libuLinux_hal.so /usr/lib/libuLinux_hal.so; do
	if [ -f "$lib" ]; then
		strings "$lib" | grep -E 'pic_|ttyS|/dev/tty|FAN|0xf[0-9a-f]' | head -80 \
			| tee "$OUT/hal_strings.txt" || true
		break
	fi
done

echo | tee -a "$OUT/summary.txt"
echo "=== dmesg uart ===" | tee -a "$OUT/summary.txt"
dmesg 2>/dev/null | grep -iE 'uart|serial|ttyS' | tail -30 | tee -a "$OUT/summary.txt" || true

echo | tee -a "$OUT/summary.txt"
echo "Capture written to $OUT"
echo "Optional (needs root): stop hal, sniff UART — see stock_strace_fan.sh"
