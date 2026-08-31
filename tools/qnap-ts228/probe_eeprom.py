#!/usr/bin/env python3
"""Probe TS-228 PIC EEPROM/MAC UART framing (stock pic_sys_get_mac fallback)."""
import sys
import time

try:
    import serial
except ImportError:
    print("need pyserial", file=sys.stderr)
    sys.exit(1)


def hexd(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


def drain(ser, t=0.3) -> bytes:
    end = time.time() + t
    data = b""
    while time.time() < end:
        chunk = ser.read(64)
        if chunk:
            data += chunk
        else:
            time.sleep(0.01)
    return data


def wait_aa(ser, tries=30) -> bool:
    for _ in range(tries):
        b = ser.read(1)
        if b in (b"\xaa", b"\x00"):
            return True
        if not b:
            time.sleep(0.05)
    return False


def eeprom_read(ser, offset: int, length: int):
    ser.reset_input_buffer()
    ser.write(b"\xf6")
    ser.flush()
    time.sleep(0.005)
    if not wait_aa(ser):
        print(f"no aa after f6 for off={offset:#x}")
        print("rx", hexd(drain(ser, 0.2)))
        return None

    ser.write(bytes([0xA1, offset & 0xFF, length & 0xFF]))
    ser.flush()
    time.sleep(0.01)
    if not wait_aa(ser):
        print(f"no aa after a1 off={offset:#x} len={length}")
        print("rx", hexd(drain(ser, 0.3)))
        return None

    data = b""
    end = time.time() + 2.0
    while len(data) < length + 1 and time.time() < end:
        chunk = ser.read(64)
        if chunk:
            data += chunk
        else:
            time.sleep(0.01)

    print(f"eeprom[{offset:#x},{length}] raw({len(data)}): {hexd(data[:80])}")
    if len(data) >= length + 1:
        payload = data[:length]
        csum = data[length]
        calc = sum(payload) & 0xFF
        print(f"  ascii={payload!r} csum={csum:#x} calc={calc:#x} ok={csum == calc}")
        return payload
    return data


def main() -> int:
    ser = serial.Serial("/dev/ttyS1", 19200, timeout=0.05)
    print("drain", hexd(drain(ser, 0.1)))
    ser.write(b"\xf2")
    ser.flush()
    print("after f2", hexd(drain(ser, 0.25)))

    for off, ln, name in (
        (0x10, 18, "mac0-18"),
        (0x10, 17, "mac0-17"),
        (0x38, 24, "sn-24"),
        (0x00, 32, "head-32"),
        (0x10, 32, "macblk-32"),
    ):
        print("===", name, "===")
        eeprom_read(ser, off, ln)
        time.sleep(0.2)

    print("=== alt a1+off only ===")
    ser.reset_input_buffer()
    ser.write(b"\xf6")
    ser.flush()
    time.sleep(0.005)
    wait_aa(ser)
    ser.write(b"\xa1\x10")
    ser.flush()
    print(hexd(drain(ser, 0.5)))

    print("=== power recovery cmds ===")
    for cmd in (0x48, 0x49, 0xFD, 0xFE):
        ser.write(bytes([cmd, cmd]))
        ser.flush()
        print(f"cmd {cmd:#x} rx", hexd(drain(ser, 0.15)))

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
