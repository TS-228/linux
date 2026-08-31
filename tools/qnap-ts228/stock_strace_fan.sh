#!/bin/bash
# Stock QTS: trace hal_daemon UART traffic while fan speed changes.
# Run as root on stock NAS after hal_daemon is running normally.
#
# In another session, change fan speed from QTS web UI or:
#   echo ... > /sys/...  (if exposed)
#
# Usage:
#   sh stock_strace_fan.sh /dev/ttyS1

TTY="${1:-/dev/ttyS1}"
PID="$(pidof hal_daemon || true)"
if [ -z "$PID" ]; then
	echo "hal_daemon not running" >&2
	exit 1
fi

echo "Tracing hal_daemon PID $PID (filter read/write on $TTY)"
echo "Change fan speed from QTS UI now, then Ctrl-C after ~30s"
strace -fp "$PID" -e trace=read,write -s 256 2>&1 | tee /tmp/hal_uart_trace.log
