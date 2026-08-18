#!/usr/bin/env bash
# Verify TS-228 front-panel GPIO buttons + disk error LEDs after DTB reboot.
#
# Usage: tools/qnap-ts228/test-gpio-frontpanel.sh [user@host]

set -euo pipefail
HOST="${1:-root@192.168.178.96}"
SSH_KEY="${SSH_KEY:-$HOME/.ssh/id_ed25519}"
SSH=(ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$HOST")

echo "=== gpio-keys / leds ==="
"${SSH[@]}" 'ls -la /sys/class/leds/; ls -la /dev/input/by-path/ 2>/dev/null || ls /dev/input/
cat /sys/kernel/debug/gpio 2>/dev/null | head -40 || true'

echo ""
echo "=== bi-color disk LEDs (on=red, off=green; 2s each) ==="
"${SSH[@]}" 'for led in red:disk1 red:disk2; do
  d=/sys/class/leds/\$led
  if [ -d \$d ]; then
    echo "\$led RED"; echo 255 > \$d/brightness; sleep 2
    echo "\$led GREEN"; echo 0 > \$d/brightness; sleep 2
  else
    echo "missing \$d (need DTB with gpio-leds)"
  fi
done'

echo ""
echo "=== press USB-copy / RESET within 10s (evtest if present) ==="
"${SSH[@]}" 'if command -v evtest >/dev/null; then
  timeout 10 evtest /dev/input/event0 2>/dev/null || timeout 10 evtest \$(ls /dev/input/event* | head -1)
else
  # poll gpio values via debugfs labels if bound
  for i in \$(seq 1 50); do
    grep -E "gpio-4[45]" /sys/kernel/debug/gpio 2>/dev/null || true
    sleep 0.2
  done | uniq
fi'
