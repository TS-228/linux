# QNAP TS-228 stock firmware reference (offline)

Pulled from stock QTS 3.10.20 on TS-228 for mainline kernel development **without**
re-booting into QTS for every test.

Refresh from a running stock NAS:

```bash
tools/qnap-ts228/pull-stock.sh admin@192.168.178.98
```

## Layout

| Path | Contents |
|------|----------|
| `configs/model_QX421_12.conf` | Model HAL config (`FAN_UNIT=PIC`, `PIC_DEV=/dev/ttyS1`) |
| `configs/hal.conf` | Enclosure / fan region IDs |
| `configs/enclosure_0.conf` | Enclosure metadata |
| `binaries/libuLinux_hal.so` | HAL library — `pic_sys_*`, `pic_handle_cmd` |
| `binaries/hal_app` | CLI used by QTS / manual fan tests |
| `binaries/hal_daemon` | `hal_util` binary (daemon entry point) |
| `logs/hal_lib.log` | PIC thread UART command log |
| `logs/hal_daemon.log` | Fan mode changes via message queue |
| `capture/summary.txt` | Automated RE capture |
| `capture/hal_strings.txt` | Full strings from HAL library |
| `analysis/hal-symbols.txt` | `nm -D` PIC/fan exports |
| `analysis/hal-pic-strings.txt` | Filtered PIC-related strings |
| `analysis/live-fan-trace.log` | Fan + PIC command excerpts |
| `analysis/fan-mode-bytes.log` | Observed fan-set UART bytes |

## Hardware paths (verified live)

### Two UARTs

| Linux device | SoC | Role |
|--------------|-----|------|
| `/dev/ttyS0` | UART0 @ 0x18007800 | **Serial console** (your debug cable) |
| `/dev/ttyS1` | UART1 @ 0x1801B200 | **Front-panel PIC** (fan, temp, LEDs, MAC, …) |

Boot cmdline on stock: `console=ttyS0,115200`

### Fan control = PIC on ttyS1 (not GPIO)

From `configs/model_QX421_12.conf`:

```
PIC_DEV = /dev/ttyS1
FAN_UNIT = PIC
```

GPIO is used for physical buttons (USB-copy, reset) and disk LEDs — **not**
the system fan (that is PIC). The front power button is not exposed to Linux
(no GPIO/PIC UART event on TS-228; stock `hal_event --se_pwb` uses eMCU/SIO).

### Stock software architecture

```
hal_app / QTS UI
  → SysV message queue
  → hal_daemon (hal_util)
  → libuLinux_hal: set_fan_speed / pic_sys_set_fan_speed
  → storage_util PIC monitor thread (opens /dev/ttyS1 at boot)
  → pic_handle_cmd(0x30..0x35) written to UART
  → front-panel microcontroller → fan
```

On stock QTS, **`storage_util --sys_startup` holds `/dev/ttyS1` open** (not `hal_daemon`
directly). On mainline, our `qnap-ts228-pic` serdev driver owns UART1 instead.

## PIC UART protocol @ 19200 8N1

| Action | TX bytes | Notes |
|--------|----------|-------|
| Init | `0xf2` | Then drain RX (~200 ms) |
| Set fan mode N | `0x30 + N` (N=0..4) | Single byte; stock mode 5 → `0x34` |
| Read fan RPM | `0xf6` → ack → `0xf7` → ack → data | RPM ≈ data × 60 |
| Read temp °C | `0xf6` → ack → `0xf8` → ack → data | data = °C |
| Status LED | `0x59` then mode | off; green=`0x56`; red=`0x57`; blink=`0x58` |
| USB LED on/blink/off | `0x60` / `0x61` / `0x62` | Front USB LED |
| Breath LED on/off | `0x2b` / `0x2a` | Present in HAL; not exposed (no visible effect on TS-228) |
| EEPROM read | `0xf6` → ack → `0xa1` + off + len → ack → data + csum | csum = sum(data) & 0xff |
| EEPROM write | `0xf6` → ack → `0xa0` + off + len → ack → data + csum → ack | MAC @ `0x10` (18), SN @ `0x38` (24, `SN:` prefix) |
| Power recovery | `0x48` (modes 0/2) or `0x49` (mode 3) | No PIC get; stock mode 1 is a no-op |
| Buzzer short/long | `0x50` / `0x51` | Vendor `QNAP_PIC_BUZZER_*` |

## Front-panel GPIO (not PIC)

Stock `model_QX421_12.conf` uses `GPIO:I2x:P` where `I2x` is the **misc GPIO pin in hex**
(exported under `/sys/class/gpio/gpiochip0`, base 0):

| Function | Config | Linux GPIO | Mainline |
|----------|--------|------------|----------|
| USB copy button | `GPIO:I2C:P` | misc 44 | `gpio-keys` → `KEY_COPY` |
| Reset button | `GPIO:I2D:P` | misc 45 | `gpio-keys` → `KEY_RESTART` |
| Power button | *(not in model)* | — | not exposed (PIC HW / stock `se_pwb`) |
| Disk1 bay LED | `GPIO:I2E:P` | misc 46 | `gpio-leds` `red:disk1` (bi-color) |
| Disk2 bay LED | `GPIO:I2F:P` | misc 47 | `gpio-leds` `red:disk2` (bi-color) |

Buttons idle high (active-low). Disk LEDs are bi-color on one GPIO each: stock
`ERR_LED` with `:P` is active-low red; line high shows green (no separate
`PRESENT_LED` in the model). Sysfs: brightness 1 = red, 0 = green.

**Buzzer** is on the PIC (`0x50` short / `0x51` long), not GPIO. Model
`VOICE_ALERT_SUPPORT = 0` only disables spoken alerts.


### Live mode → byte mapping (192.168.178.98)

| `hal_app` mode | UART byte |
|----------------|-----------|
| 0 | `0x30` |
| 1 | `0x31` |
| 5 | `0x34` |

Stock command:

```sh
hal_app --se_sys_set_fan_mode enc_sys_id=root,obj_index=0,mode=5
```

Log line: `pic_handle_cmd: Server receive command = 0x34`

## Mainline workflow (no stock boot)

### 1. Kernel driver (preferred)

```bash
# after one modular-kernel boot:
tools/qnap-ts228/reload-pic.sh root@<mainline-host>
tools/qnap-ts228/mainline-pic-test.sh root@<mainline-host>
```

Sysfs: `/sys/class/hwmon/hwmon*/{pwm1,fan1_input,temp1_input}`

PIC device attrs (on the serdev): `led_status`, `led_usb`, `serial_number`,
`mac_address`, `power_recovery` under `/sys/bus/serial/devices/serial0-0/`.

### 2. Userspace UART probe (driver unloaded)

Build and deploy:

```bash
make -C tools/qnap-ts228 uart_probe
scp tools/qnap-ts228/uart_probe root@<host>:/tmp/
# on NAS — only if nothing else owns ttyS1:
/tmp/uart_probe init
/tmp/uart_probe fan 5
/tmp/uart_probe read-fan
/tmp/uart_probe read-temp
```

### 3. Offline HAL analysis (this directory)

```bash
nm -D stock-firmware/binaries/libuLinux_hal.so | grep pic_
strings stock-firmware/binaries/libuLinux_hal.so | grep -i pic
# optional: r2, ghidra on libuLinux_hal.so
```

## Key HAL symbols

See `analysis/hal-symbols.txt`. Important entry points:

- `pic_sys_start_monitor` — opens PIC UART, init thread
- `pic_handle_cmd` — dispatches command byte to UART
- `pic_sys_set_fan_speed` / `pic_sys_get_fan_speed`
- `pic_sys_get_temp`

## Stock-only commands (reference)

Run on QTS only — require `hal_daemon` + `storage_util` PIC thread:

```sh
hal_app --se_sys_set_fan_mode enc_sys_id=root,obj_index=0,mode=N
tail -f /var/log/hal_lib.log    # watch pic_handle_cmd lines
/usr/sbin/lsof /dev/ttyS1       # expect storage_util
```

## License note

Binaries under `binaries/` are extracted from QTS firmware for reverse-engineering
reference only. Do not redistribute; refresh with `pull-stock.sh` from your own device.
