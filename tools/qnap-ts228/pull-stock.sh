#!/usr/bin/env bash
# Pull stock QTS artifacts from a running TS-228 for offline mainline development.
#
# Usage:
#   tools/qnap-ts228/pull-stock.sh [user@host]
#
# Default host: admin@192.168.178.98
# Output: tools/qnap-ts228/stock-firmware/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SCRIPT_DIR/stock-firmware"
HOST="${1:-admin@192.168.178.98}"
SSH=(ssh -o StrictHostKeyChecking=no "$HOST")
SCP=(scp -o StrictHostKeyChecking=no)

mkdir -p "$OUT"/{capture,configs,binaries,logs,analysis}

echo "Pulling from $HOST into $OUT ..."

# Run capture script if present (non-fatal)
"${SSH[@]}" 'test -x /tmp/stock_hal_re.sh && sh /tmp/stock_hal_re.sh /tmp/qnap-hal-re || true' 2>/dev/null || true

pull() {
	local remote="$1" local="$2"
	if "${SSH[@]}" "test -e '$remote'" 2>/dev/null; then
		echo "  $remote"
		"${SCP[@]}" "$HOST:$remote" "$local"
	fi
}

echo "Configs ..."
pull /etc/model_QX421_12.conf "$OUT/configs/model_QX421_12.conf"
pull /etc/hal.conf "$OUT/configs/hal.conf"
pull /etc/enclosure_0.conf "$OUT/configs/enclosure_0.conf"
"${SSH[@]}" 'readlink -f /etc/model.conf 2>/dev/null || true' | tee "$OUT/configs/model.conf.symlink"

echo "Capture ..."
pull /tmp/qnap-hal-re/summary.txt "$OUT/capture/summary.txt"
pull /tmp/qnap-hal-re/hal_strings.txt "$OUT/capture/hal_strings.txt"

echo "Logs ..."
pull /var/log/hal_lib.log "$OUT/logs/hal_lib.log"
pull /var/log/hal_daemon.log "$OUT/logs/hal_daemon.log"
pull /var/log/hal_util_net.log "$OUT/logs/hal_util_net.log"

echo "Binaries (for offline symbol/strings analysis) ..."
pull /lib/libuLinux_hal.so "$OUT/binaries/libuLinux_hal.so"
pull /sbin/hal_app "$OUT/binaries/hal_app"
pull /sbin/hal_util "$OUT/binaries/hal_daemon"

echo "Local analysis ..."
if [ -f "$OUT/binaries/libuLinux_hal.so" ]; then
	nm -D "$OUT/binaries/libuLinux_hal.so" 2>/dev/null \
		| grep -E 'pic_|set_fan|FAN' | sort -u > "$OUT/analysis/hal-symbols.txt" || true
	strings "$OUT/binaries/libuLinux_hal.so" \
		| grep -E '^pic_|^se_sys_.*fan|Write to PIC|pic_handle|/dev/ttyS' \
		| sort -u > "$OUT/analysis/hal-pic-strings.txt" || true
fi
if [ -f "$OUT/logs/hal_lib.log" ]; then
	grep -E 'pic_handle_cmd|Write to PIC|pic_sys_start|set_fan_speed|Change Fan' \
		"$OUT/logs/hal_lib.log" "$OUT/logs/hal_daemon.log" 2>/dev/null \
		| tail -100 > "$OUT/analysis/live-fan-trace.log" || true
	grep -E 'pic_handle_cmd.*0x3' "$OUT/logs/hal_lib.log" | sort -u \
		> "$OUT/analysis/fan-mode-bytes.log" 2>/dev/null || true
fi

{
	echo "pulled: $(date -Iseconds)"
	echo "host: $HOST"
	sha256sum "$OUT"/configs/* "$OUT"/capture/* "$OUT"/binaries/* 2>/dev/null || true
} > "$OUT/MANIFEST.txt"

echo "Done. See $OUT/README.md"
