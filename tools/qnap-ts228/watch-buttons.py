#!/usr/bin/env python3
"""Print TS-228 front-panel button presses (gpio-keys).

Listens on all /dev/input/event* by default:
  - gpio-keys: usb-copy (KEY_COPY), reset (KEY_RESTART)

The front power button is not exposed as an input event on TS-228.

Usage:
  tools/qnap-ts228/watch-buttons.py [event-device ...]
"""

from __future__ import annotations

import errno
import os
import select
import struct
import sys
import time

EV_KEY = 0x01

KEY_NAMES = {
    133: "usb-copy (KEY_COPY)",
    408: "reset (KEY_RESTART)",
}


def event_fmt() -> str:
    import platform

    return "llHHi" if platform.architecture()[0].startswith("32") else "qqHHi"


def list_event_devices() -> list[str]:
    base = "/dev/input"
    return sorted(
        os.path.join(base, name)
        for name in os.listdir(base)
        if name.startswith("event")
    )


def device_name(path: str) -> str:
    # /dev/input/eventN -> /sys/class/input/eventN/device/name
    num = os.path.basename(path)
    sysfs = f"/sys/class/input/{num}/device/name"
    try:
        with open(sysfs, encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return path


def main() -> int:
    paths = sys.argv[1:] or list_event_devices()
    if not paths:
        print("No /dev/input/event* devices found", file=sys.stderr)
        return 1

    fmt = event_fmt()
    size = struct.calcsize(fmt)
    files = []
    fd_map = {}

    for path in paths:
        try:
            f = open(path, "rb", buffering=0)
        except OSError as e:
            print(f"skip {path}: {e}", file=sys.stderr)
            continue
        files.append(f)
        fd_map[f.fileno()] = (path, device_name(path), f)

    if not files:
        print("No readable input devices", file=sys.stderr)
        return 1

    print("Listening on:", flush=True)
    for path, name, _ in fd_map.values():
        print(f"  {path}  ({name})", flush=True)
    print("Press USB-copy / RESET; Ctrl-C to stop.", flush=True)
    print("(Power button is not exposed as an input event.)", flush=True)

    try:
        while True:
            r, _, _ = select.select(files, [], [], 1.0)
            for f in r:
                path, name, _ = fd_map[f.fileno()]
                data = f.read(size)
                if len(data) != size:
                    continue
                _sec, _usec, etype, code, value = struct.unpack(fmt, data)
                if etype != EV_KEY:
                    continue
                key = KEY_NAMES.get(code, f"KEY_{code}")
                state = {0: "release", 1: "press", 2: "repeat"}.get(value, str(value))
                ts = time.strftime("%H:%M:%S")
                print(f"{ts}  [{name}] {key}: {state}", flush=True)
    finally:
        for f in files:
            f.close()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        raise SystemExit(0)
    except OSError as e:
        if e.errno in (errno.ENODEV, errno.ENOENT):
            print(f"No input device: {e}", file=sys.stderr)
            raise SystemExit(1)
        raise
