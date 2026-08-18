#!/bin/sh
# Reverse-engineer hal_daemon fan path on stock QTS (TS-228).
# Run as root on the NAS (serial console or SSH).
#
# Answers:
#   - Is the fan driven by UART (/dev/ttyS1 PIC) or SoC GPIO/PWM?
#   - Which process owns which serial port (console ttyS0 vs PIC ttyS1)?
#
# Usage:
#   sh stock_hal_re.sh [output_dir]
#
# Optional live proof (second session, root):
#   sh stock_hal_re.sh /tmp/hal-re
#   sh stock_hal_re.sh trace-fan    # strace while you change fan mode

set -eu

OUT="${1:-/tmp/qnap-hal-re}"
MODE="${1:-}"

log() { echo "$@" | tee -a "$OUT/summary.txt"; }

run_capture() {
	mkdir -p "$OUT"
	: > "$OUT/summary.txt"

	log "=== QNAP TS-228 hal_daemon fan path ==="
	log "date: $(date 2>/dev/null || true)"
	log "uname: $(uname -a 2>/dev/null || true)"
	log ""

	log "=== boot console (should be ttyS0 / UART0, NOT the PIC) ==="
	grep -E 'console=|stdout-path|earlycon' /proc/cmdline 2>/dev/null | tee -a "$OUT/summary.txt" || true
	log ""

	log "=== model.conf (FAN_UNIT / PIC device) ==="
	for f in /etc/model.conf /etc/default/model.conf; do
		if [ -f "$f" ]; then
			log "--- $f ---"
			grep -E 'PIC|FAN|MCU|UART|ttyS|GPIO|PWM' "$f" 2>/dev/null | tee -a "$OUT/summary.txt" || true
		fi
	done
	log ""

	log "=== hal_daemon.cfg ==="
	for f in /etc/default/hal_daemon.cfg /etc/hal_daemon.cfg; do
		if [ -f "$f" ]; then
			log "--- $f ---"
			grep -E 'PIC|FAN|MCU|UART|ttyS|GPIO' "$f" 2>/dev/null | tee -a "$OUT/summary.txt" || true
		fi
	done
	log ""

	log "=== hal.conf enclosure (if present) ==="
	if [ -f /etc/hal.conf ]; then
		grep -E 'FAN|fan|enc_sys' /etc/hal.conf 2>/dev/null | head -40 | tee -a "$OUT/summary.txt" || true
	fi
	log ""

	log "=== hal processes ==="
	ps w | grep -E '[h]al_daemon|[h]al_app|[h]al_event|[h]al_tool' | tee -a "$OUT/summary.txt" || true
	log ""

	HALPID="$(pidof hal_daemon 2>/dev/null || true)"
	log "hal_daemon pid: ${HALPID:-<not running>}"
	log ""

	log "=== serial devices ==="
	ls -la /dev/ttyS* 2>&1 | tee -a "$OUT/summary.txt" || true
	log ""

	log "=== who owns ttyS0 (Linux console) ==="
	lsof /dev/ttyS0 2>&1 | tee -a "$OUT/summary.txt" || true
	log ""

	log "=== who owns ttyS1 (expected: hal_daemon PIC) ==="
	lsof /dev/ttyS1 2>&1 | tee -a "$OUT/summary.txt" || true
	log ""

	if [ -n "$HALPID" ]; then
		log "=== hal_daemon open file descriptors ==="
		ls -la "/proc/$HALPID/fd" 2>&1 | tee -a "$OUT/summary.txt" || true
		log ""
		log "--- fd targets (grep tty/gpio) ---"
		for fd in /proc/"$HALPID"/fd/*; do
			target=$(readlink "$fd" 2>/dev/null || true)
			case "$target" in
			*tty*|*gpio*) log "$fd -> $target" ;;
			esac
		done | tee -a "$OUT/summary.txt"
		log ""
	fi

	log "=== dmesg uart (kernel map) ==="
	dmesg 2>/dev/null | grep -iE '18007800|1801[Bb]200|ttyS|uart|serial' | tail -40 | tee -a "$OUT/summary.txt" || true
	log ""

	log "=== GPIO / PWM sysfs (fan usually NOT here on TS-228) ==="
	ls -d /sys/class/gpio/gpio* 2>/dev/null | head -20 | tee -a "$OUT/summary.txt" || log "(no exported gpio)"
	ls -d /sys/class/pwm/pwmchip* 2>/dev/null | tee -a "$OUT/summary.txt" || log "(no pwmchip)"
	log ""

	HAL_LIB=""
	for lib in /lib/libuLinux_hal.so /usr/lib/libuLinux_hal.so; do
		if [ -f "$lib" ]; then HAL_LIB="$lib"; break; fi
	done

	if [ -n "$HAL_LIB" ]; then
		log "=== libuLinux_hal.so static hints ($HAL_LIB) ==="
		strings "$HAL_LIB" > "$OUT/hal_strings.txt"
		grep -E 'pic_|PIC|ttyS|/dev/tty|se_sys_set_fan|FAN_UNIT|gpio|GPIO|pwm|PWM|0x30|0xf2|0xf6|0xf7|0xf8|0x51' \
			"$OUT/hal_strings.txt" | sort -u | head -120 | tee -a "$OUT/summary.txt"
		log ""
		if command -v nm >/dev/null 2>&1; then
			log "--- nm pic/fan/gpio symbols ---"
			nm -D "$HAL_LIB" 2>/dev/null | grep -E 'pic_|fan|gpio|pwm|FAN' | head -60 | tee -a "$OUT/summary.txt" || true
		fi
		if command -v readelf >/dev/null 2>&1; then
			log "--- readelf NEEDED ---"
			readelf -d "$HAL_LIB" 2>/dev/null | grep NEEDED | tee -a "$OUT/summary.txt" || true
		fi
		log ""
	fi

	HAL_BIN=""
	for bin in /sbin/hal_daemon /usr/sbin/hal_daemon; do
		if [ -x "$bin" ]; then HAL_BIN="$bin"; break; fi
	done
	if [ -n "$HAL_BIN" ] && command -v strings >/dev/null 2>&1; then
		strings "$HAL_BIN" | grep -E 'pic_|ttyS|/dev/tty|msgget|msgrcv|msgsnd|FAN_UNIT|GPIO' | sort -u | head -40 \
			| tee -a "$OUT/summary.txt" || true
		log ""
	fi

	log "=== hal_app fan commands (help strings) ==="
	for app in /sbin/hal_app /usr/sbin/hal_app /sbin/hal_event /usr/sbin/hal_event; do
		if [ -x "$app" ]; then
			log "--- $app ---"
			strings "$app" 2>/dev/null | grep -iE 'fan_mode|fan_pwm|fan_scroll|set_fan' | head -20 | tee -a "$OUT/summary.txt" || true
		fi
	done
	log ""

	log "=== current fan sysfs / HAL readouts ==="
	for f in /tmp/FANS_0 /tmp/FAN_* /var/run/FANS_0; do
		[ -f "$f" ] && log "$f: $(cat "$f" 2>/dev/null)" || true
	done
	for cmd in /sbin/hal_get_fan_speed /usr/sbin/hal_get_fan_speed; do
		if [ -x "$cmd" ]; then
			log "# $cmd"
			"$cmd" 2>&1 | tee -a "$OUT/summary.txt" || true
		fi
	done
	log ""

	log "=== CONCLUSION HINTS (read model.conf + lsof above) ==="
	log "TS-228 expected:"
	log "  - console = /dev/ttyS0 (UART @ 0x18007800) — your serial cable"
	log "  - PIC/fan = /dev/ttyS1 (UART @ 0x1801B200) — hal_daemon only"
	log "  - model.conf: FAN_UNIT=PIC, PIC_DEV=/dev/ttyS1"
	log "  - fan bytes on UART: 0x30..0x35 (not SoC GPIO)"
	log ""
	log "Capture saved under: $OUT"
	log "Next: sh $(basename "$0") trace-fan   # while changing fan in UI"
}

run_trace_fan() {
	PID="$(pidof hal_daemon || true)"
	if [ -z "$PID" ]; then
		echo "hal_daemon not running" >&2
		exit 1
	fi

	OUT="/tmp/qnap-hal-re"
	mkdir -p "$OUT"
	TRACE="$OUT/hal_daemon_strace.log"

	echo "Tracing hal_daemon PID $PID -> $TRACE"
	echo "In another session run ONE of:"
	echo "  hal_app --se_sys_set_fan_mode enc_id=0,mode=0   # low/auto"
	echo "  hal_app --se_sys_set_fan_mode enc_id=0,mode=5   # full"
	echo "  (or change fan in QTS web UI)"
	echo "Ctrl-C after ~20s"

	# Show only syscalls that touch serial or gpio
	strace -f -p "$PID" -s 256 -yy \
		-e trace=open,openat,read,write,ioctl,close \
		2>&1 | tee "$TRACE" &
	STRACE_PID=$!

	sleep 20
	kill "$STRACE_PID" 2>/dev/null || true
	wait "$STRACE_PID" 2>/dev/null || true

	echo ""
	echo "=== ttyS / gpio lines from trace ==="
	grep -E 'ttyS|gpio|/dev/' "$TRACE" | head -80 || true
	echo ""
	echo "=== write() payloads (fan set often single byte 0x3x) ==="
	grep '= [0-9]+$' "$TRACE" | grep write | head -40 || true
	echo "Full log: $TRACE"
}

case "$MODE" in
trace-fan) run_trace_fan ;;
*) run_capture ;;
esac
