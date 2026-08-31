// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hwmon + LED driver for the QNAP TS-228 front-panel PIC on UART1.
 *
 * Protocol reverse-engineered from QTS 3.10 libuLinux_hal.so (pic_sys_*):
 *  - 19200 8N1 raw serial (stock tcsetattr c_cflag B19200|CS8|CREAD|CLOCAL)
 *  - No init handshake: stock opens the port and polls it. 0xf2 is
 *    QNAP_PIC_WOL_ENABLE, not an init command -- do not send it.
 *  - Fan set: single byte 0x30..0x34 (pic_sys_set_fan_speed table; mode 5 = 0x34)
 *  - LEDs (se_sys_set_*_led PIC backend; status is one bi-color LED):
 *      status: off 0x59; green 0x56; red 0x57; blink 0x58
 *      USB on 0x60; blink 0x61; off 0x62
 *  - EEPROM: 0xf6 sync, wait 0xaa, drain, then 0xa1/0xa0 + offset + len
 *      (+ data + csum for write), wait 0xaa, then payload + checksum.
 *      MAC is 18 bytes of ASCII at 0x10, board SN 24 bytes at 0x38.
 *      Stock retries the 0xaa waits 12 times at 1 s each.
 *  - Power recovery: 0x48 (modes 0/2), 0x49 (mode 3)
 *  - Buzzer (vendor include/qnap/pic.h): short 0x50, long 0x51. Exposed to
 *      userspace as EV_SND (SND_BELL=short, SND_TONE=long) on a plain input
 *      device, the same ABI mainline's qnap-mcu-input.c uses for the
 *      TS-233/TS-433 MCU beeper -- not a bespoke sysfs attribute.
 *  - Software shutdown: 0x41 (QNAP_PIC_SOFTWARE_SHUTDOWN); cuts PSU
 *  - Fan RPM readback: 0xf6 sync, wait 0xaa, 0xf7, wait 0xaa, one data byte
 *      (RPM = data * 60). Board-verified reliable, 5/5 identical reads.
 *      Temperature readback (0xf8 in place of 0xf7) replies the same way
 *      but the data byte is always 0x00, including polled once a minute
 *      out to 11+ minutes uptime -- not a probe-time/warm-up artifact.
 *      Confirmed via stock QTS: hal_app's own reported "system temp" tracks
 *      /sys/class/thermal/thermal_zone0/temp (the SoC's rtk_thermal zone)
 *      in lockstep, not the PIC -- despite model.conf's SYSTEM_TEMP_UNIT=PIC,
 *      stock itself never gets a real reading from this command either.
 *      Not exposed via hwmon.
 *  - The full 0xf6-sync subcommand byte space (0x00-0xff, minus 0xa0/0xa1
 *      which are the known EEPROM write/read) has been board-swept. Every
 *      value NACKs cleanly (cmd-ack failure, no side effect) except one:
 *  - WARNING, board-verified the hard way: subcommand 0x55 gets acked but
 *      never returns a data byte, and wedges the PIC afterward -- every
 *      subsequent 0xf6 sync stops getting acked, on both this driver and
 *      stock QTS's own hal_daemon (confirmed via its hal_lib.log:
 *      pic_sys_get_mac starts failing with "pic_thead not exist"). A full
 *      relay-controlled power cycle of the board, including a 45 s hard-off,
 *      does NOT clear this -- the PIC evidently has its own standby supply
 *      that a downstream relay doesn't remove. Only pulling the AC adapter
 *      itself recovered it. 0x55 is presumably a real multi-byte command
 *      (like 0xa0/0xa1) whose payload we're truncating by not sending
 *      follow-up bytes, but that payload shape is unknown -- do not send it,
 *      or any other undocumented subcommand, outside of a deliberately
 *      scoped, closely-watched experiment with power-cycle recovery in hand.
 *
 * Soft KEY_POWER is not available: the front power button does not send a
 * UART event on TS-228 (stock uses se_pwb / eMCU-SIO; long-press is PIC HW).
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/hwmon.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/leds.h>
#include <linux/serial.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/thermal.h>

#define QNAP_TS228_PIC_BAUDRATE		19200
#define QNAP_TS228_PIC_TIMEOUT_MS	1000
#define QNAP_TS228_PIC_RX_SIZE		256

#define PIC_CMD_SYNC			0xf6
#define PIC_CMD_ACK			0xaa
#define PIC_CMD_FAN_LEVELS		5
#define PIC_READ_RETRIES		3
#define PIC_FAN_HOLDOFF_MS		10000

#define PIC_CMD_STATUS_OFF		0x59
#define PIC_CMD_STATUS_GREEN		0x56
#define PIC_CMD_STATUS_RED		0x57
#define PIC_CMD_STATUS_BLINK		0x58
#define PIC_CMD_USB_ON			0x60
#define PIC_CMD_USB_BLINK		0x61
#define PIC_CMD_USB_OFF			0x62
#define PIC_CMD_EEPROM_WRITE		0xa0
#define PIC_CMD_EEPROM_READ		0xa1
#define PIC_CMD_SOFTWARE_SHUTDOWN	0x41
#define PIC_CMD_PWR_RECOVERY_A		0x48
#define PIC_CMD_PWR_RECOVERY_B		0x49
#define PIC_CMD_BUZZER_SHORT		0x50
#define PIC_CMD_BUZZER_LONG		0x51

/*
 * Unsolicited status push the PIC sends on its own ~5s timer, completely
 * outside of any command/response exchange. The 2-byte prefix is constant
 * across boots; the trailing byte differed between two observed boots
 * (0xa2, 0xa3) but stayed constant across many cycles within a boot -
 * likely a boot-scoped status/event code, not a counter. Not yet decoded
 * further, but the vendor HAL (libuLinux_hal.so) has its own "heartbeat"
 * and "unsolicited data" concepts, so this is a real, expected protocol
 * message rather than noise - see qnap_ts228_pic_receive_buf().
 */
#define PIC_HEARTBEAT_B0		0x74
#define PIC_HEARTBEAT_B1		0x75

#define PIC_EEPROM_MAC_OFF		0x10
#define PIC_EEPROM_MAC_LEN		18
#define PIC_EEPROM_SN_OFF		0x38
#define PIC_EEPROM_SN_LEN		24
#define PIC_EEPROM_MAX_LEN		64

/* Stock pic_sys_set_power_recovery_mode: 0/2 → 0x48, 3 → 0x49 */
/* Vendor QNAP_PIC_BUZZER_*: short 0x50, long 0x51 */
/* pic_sys_set_fan_speed() jump table in libuLinux_hal.so */
static const u8 pic_fan_cmd[PIC_CMD_FAN_LEVELS] = {
	0x30, 0x31, 0x32, 0x33, 0x34,
};

/* hwmon pwm1_enable values (Documentation/hwmon/pwmfan.rst) */
#define QNAP_PIC_PWM_OFF		0
#define QNAP_PIC_PWM_MANUAL		1
#define QNAP_PIC_PWM_AUTO		2

/* led_status sysfs: bi-color status LED modes */
#define QNAP_STATUS_OFF			0
#define QNAP_STATUS_GREEN		1
#define QNAP_STATUS_RED			2
#define QNAP_STATUS_BLINK		3

enum qnap_ts228_led_id {
	QNAP_LED_STATUS = 0,
	QNAP_LED_USB,
};

struct qnap_ts228_pic {
	struct serdev_device *serdev;
	struct device *hwmon_dev;
	struct mutex lock;

	/*
	 * rx_buf/rx_len are filled from the serdev receive callback, which runs
	 * in the tty layer's context, and drained from the command path in
	 * process context. rx_lock keeps the two apart; pic->lock only
	 * serialises commands against each other and cannot be taken by the
	 * callback.
	 */
	spinlock_t rx_lock;
	u8 rx_buf[QNAP_TS228_PIC_RX_SIZE];
	unsigned int rx_len;
	struct completion rx_done;

	/* Last (still undecoded) trailing byte of the unsolicited heartbeat
	 * push - see qnap_ts228_pic_receive_buf(). rx_lock-protected, same
	 * as rx_buf/rx_len.
	 */
	u8 last_heartbeat;

	unsigned int fan_level;
	unsigned int fan_max_level;

	u8 mac_addr[ETH_ALEN];
	bool mac_valid;

	struct fwnode_handle *fan_node;
	unsigned int *fan_cooling_levels;
	unsigned int fan_max_state;
	unsigned int fan_state;

	struct thermal_cooling_device *cdev;
	struct hwmon_chip_info info;
	struct notifier_block reboot_nb;
	bool monitor_ready;
	bool pwm_auto;
	unsigned long fan_holdoff_until;

	struct led_classdev led_status_red;
	struct led_classdev led_status_green;
	struct led_classdev led_usb;
	bool status_red_on;
	bool status_green_on;
	bool status_red_blink;
	bool status_green_blink;

	struct input_dev *input;
	struct work_struct beep_work;
	int beep_type;

	unsigned int power_recovery;  /* last written mode (0/2/3) */
};

static void qnap_ts228_pic_reset_rx(struct qnap_ts228_pic *pic)
{
	unsigned long flags;

	spin_lock_irqsave(&pic->rx_lock, flags);
	pic->rx_len = 0;
	reinit_completion(&pic->rx_done);
	spin_unlock_irqrestore(&pic->rx_lock, flags);
}

/* Take the oldest byte; false when nothing is buffered. */
static bool qnap_ts228_pic_pop_rx(struct qnap_ts228_pic *pic, u8 *val)
{
	unsigned long flags;
	bool got = false;

	spin_lock_irqsave(&pic->rx_lock, flags);
	if (pic->rx_len) {
		*val = pic->rx_buf[0];
		if (--pic->rx_len)
			memmove(pic->rx_buf, pic->rx_buf + 1, pic->rx_len);
		else
			reinit_completion(&pic->rx_done);
		got = true;
	}
	spin_unlock_irqrestore(&pic->rx_lock, flags);

	return got;
}

static bool qnap_ts228_pic_rx_pending(struct qnap_ts228_pic *pic)
{
	unsigned long flags;
	bool pending;

	spin_lock_irqsave(&pic->rx_lock, flags);
	pending = pic->rx_len != 0;
	spin_unlock_irqrestore(&pic->rx_lock, flags);

	return pending;
}

static size_t qnap_ts228_pic_receive_buf(struct serdev_device *serdev,
					 const u8 *data, size_t count)
{
	struct qnap_ts228_pic *pic = serdev_device_get_drvdata(serdev);
	unsigned long flags;
	size_t i;
	bool have_data;
	bool got_heartbeat = false;
	u8 heartbeat_byte = 0;

	spin_lock_irqsave(&pic->rx_lock, flags);
	for (i = 0; i < count; i++) {
		if (pic->rx_len >= QNAP_TS228_PIC_RX_SIZE)
			break;

		pic->rx_buf[pic->rx_len++] = data[i];

		/*
		 * Recognise the unsolicited heartbeat push by its trailing
		 * edge in rx_buf, regardless of whatever else is already
		 * buffered: once its 2-byte prefix plus a 3rd byte lands at
		 * the tail, strip those 3 bytes back out. Real bytes are
		 * always appended first and only retroactively removed on a
		 * full match, so unlike gating on "rx_buf is empty" this
		 * can't silently lose a byte on a false start, and self-heals
		 * if a real command and a heartbeat ever race and leave
		 * residue instead of latching a permanent mismatch.
		 */
		if (pic->rx_len >= 3 &&
		    pic->rx_buf[pic->rx_len - 3] == PIC_HEARTBEAT_B0 &&
		    pic->rx_buf[pic->rx_len - 2] == PIC_HEARTBEAT_B1) {
			heartbeat_byte = pic->rx_buf[pic->rx_len - 1];
			pic->rx_len -= 3;
			pic->last_heartbeat = heartbeat_byte;
			got_heartbeat = true;
		}
	}
	have_data = pic->rx_len != 0;
	spin_unlock_irqrestore(&pic->rx_lock, flags);

	if (i < count)
		dev_warn_ratelimited(&serdev->dev, "rx buffer full, dropped %zu bytes\n", count - i);

	if (got_heartbeat)
		dev_dbg(&serdev->dev, "unsolicited status push: 0x%02x\n", heartbeat_byte);

	if (have_data) {
		dev_dbg(&serdev->dev, "rx (%zu): %*ph\n", count, (int)count, data);
		complete(&pic->rx_done);
	}

	return count;
}

static const struct serdev_device_ops qnap_ts228_pic_serdev_ops = {
	.receive_buf = qnap_ts228_pic_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static int qnap_ts228_pic_write(struct qnap_ts228_pic *pic, const u8 *buf,
				size_t len)
{
	int ret;

	ret = serdev_device_write(pic->serdev, buf, len,
				  msecs_to_jiffies(QNAP_TS228_PIC_TIMEOUT_MS));
	if (ret < 0)
		return ret;
	if (ret != len)
		return -EIO;

	serdev_device_wait_until_sent(pic->serdev,
				      msecs_to_jiffies(QNAP_TS228_PIC_TIMEOUT_MS));
	return 0;
}

static int qnap_ts228_pic_wait_rx(struct qnap_ts228_pic *pic,
				  unsigned int timeout_ms)
{
	unsigned long ret;

	/* Bytes may have arrived before we wait; do not reinit_completion here. */
	if (qnap_ts228_pic_rx_pending(pic))
		return 0;

	ret = wait_for_completion_timeout(&pic->rx_done,
					  msecs_to_jiffies(timeout_ms));

	return ret ? 0 : -ETIMEDOUT;
}

static int qnap_ts228_pic_wait_idle(struct qnap_ts228_pic *pic)
{
	u8 b;
	unsigned long deadline = jiffies + msecs_to_jiffies(500);

	while (time_before(jiffies, deadline)) {
		if (!qnap_ts228_pic_pop_rx(pic, &b)) {
			qnap_ts228_pic_wait_rx(pic, 50);
			continue;
		}

		if (b == 0xe0)
			return 0;
	}

	return -ETIMEDOUT;
}

static int qnap_ts228_pic_wait_ack(struct qnap_ts228_pic *pic)
{
	u8 b;
	unsigned long deadline = jiffies + msecs_to_jiffies(QNAP_TS228_PIC_TIMEOUT_MS);

	while (time_before(jiffies, deadline)) {
		if (!qnap_ts228_pic_pop_rx(pic, &b)) {
			qnap_ts228_pic_wait_rx(pic, 50);
			continue;
		}

		if (b == PIC_CMD_ACK || b == 0x00)
			return 0;
	}

	return -ETIMEDOUT;
}

static int qnap_ts228_pic_wait_byte(struct qnap_ts228_pic *pic, u8 *val)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(QNAP_TS228_PIC_TIMEOUT_MS);

	while (time_before(jiffies, deadline)) {
		if (qnap_ts228_pic_pop_rx(pic, val))
			return 0;

		qnap_ts228_pic_wait_rx(pic, 50);
	}

	return -ETIMEDOUT;
}

/* Fan/temp path: skip framing noise. EEPROM uses wait_byte (payload may be 0xaa). */
static int qnap_ts228_pic_drain(struct qnap_ts228_pic *pic, unsigned int timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	u8 b;

	while (time_before(jiffies, deadline)) {
		if (qnap_ts228_pic_pop_rx(pic, &b))
			continue;
		if (qnap_ts228_pic_wait_rx(pic, 20))
			break;
	}

	return 0;
}

static int qnap_ts228_pic_start_monitor(struct qnap_ts228_pic *pic)
{
	if (pic->monitor_ready)
		return 0;

	/*
	 * The PIC needs no initialisation: stock opens the port, sets termios
	 * and then only polls it. Just drop anything already in the FIFO so a
	 * stale event byte cannot be mistaken for a reply.
	 */
	qnap_ts228_pic_drain(pic, 20);
	qnap_ts228_pic_reset_rx(pic);
	pic->monitor_ready = true;

	return 0;
}

static int qnap_ts228_pic_set_fan_level(struct qnap_ts228_pic *pic, unsigned int level)
{
	u8 cmd;
	int ret;

	if (level >= PIC_CMD_FAN_LEVELS)
		return -EINVAL;

	cmd = pic_fan_cmd[level];

	qnap_ts228_pic_wait_idle(pic);

	ret = qnap_ts228_pic_write(pic, &cmd, 1);
	if (ret)
		return ret;

	usleep_range(50000, 80000);
	ret = qnap_ts228_pic_write(pic, &cmd, 1);
	if (ret)
		return ret;

	usleep_range(200000, 250000);
	qnap_ts228_pic_reset_rx(pic);

	pic->fan_holdoff_until = jiffies + msecs_to_jiffies(PIC_FAN_HOLDOFF_MS);
	dev_dbg(&pic->serdev->dev, "fan level %u -> 0x%02x (x2)\n", level, cmd);
	return 0;
}

/* Fire-and-forget PIC cmds (LEDs) are lossy; mirror fan: idle + send x2. */
static int qnap_ts228_pic_send_cmd(struct qnap_ts228_pic *pic, u8 cmd)
{
	int ret;

	qnap_ts228_pic_wait_idle(pic);

	ret = qnap_ts228_pic_write(pic, &cmd, 1);
	if (ret)
		return ret;

	usleep_range(50000, 80000);
	ret = qnap_ts228_pic_write(pic, &cmd, 1);
	if (ret)
		return ret;

	usleep_range(30000, 50000);
	qnap_ts228_pic_reset_rx(pic);
	return 0;
}

/*
 * Reboot notifiers run before device_shutdown(), while UART/serdev still
 * work. POWER_OFF_PREPARE is too late for a reliable PIC command.
 * Stock opcode: QNAP_PIC_SOFTWARE_SHUTDOWN (0x41).
 */
static int qnap_ts228_pic_reboot_notify(struct notifier_block *nb,
					unsigned long mode, void *cmd)
{
	struct qnap_ts228_pic *pic =
		container_of(nb, struct qnap_ts228_pic, reboot_nb);
	int ret;

	if (mode != SYS_POWER_OFF)
		return NOTIFY_DONE;

	mutex_lock(&pic->lock);
	ret = qnap_ts228_pic_send_cmd(pic, PIC_CMD_SOFTWARE_SHUTDOWN);
	mutex_unlock(&pic->lock);
	if (ret) {
		dev_err(&pic->serdev->dev, "PIC software shutdown failed: %d\n",
			ret);
		return NOTIFY_DONE;
	}

	dev_info(&pic->serdev->dev, "PIC software shutdown (0x41) sent\n");
	/* Allow the PIC to cut the PSU before SoC coolboot runs. */
	mdelay(1500);
	return NOTIFY_OK;
}

static void qnap_ts228_pic_shutdown(struct serdev_device *serdev)
{
	struct qnap_ts228_pic *pic = serdev_device_get_drvdata(serdev);

	if (!pic)
		return;

	/*
	 * device_shutdown() runs this unconditionally for reboot, halt, and
	 * poweroff alike -- unlike the reboot notifier above, it gets no
	 * mode argument. Only cut the PSU on an actual poweroff/halt; on
	 * SYSTEM_RESTART this must be a no-op or "reboot" cuts power via the
	 * PIC instead of letting the SoC reset, and the board just stays off.
	 */
	if (system_state != SYSTEM_POWER_OFF && system_state != SYSTEM_HALT)
		return;

	mutex_lock(&pic->lock);
	qnap_ts228_pic_send_cmd(pic, PIC_CMD_SOFTWARE_SHUTDOWN);
	mutex_unlock(&pic->lock);
	dev_emerg(&serdev->dev, "PIC software shutdown (0x41) via device shutdown\n");
	mdelay(1500);
}

static void qnap_ts228_pic_unregister_reboot(void *data)
{
	struct qnap_ts228_pic *pic = data;

	unregister_reboot_notifier(&pic->reboot_nb);
}

static int qnap_ts228_pic_eeprom_read(struct qnap_ts228_pic *pic, u8 offset,
				      u8 *buf, u8 len)
{
	u8 stale;
	u8 sync = PIC_CMD_SYNC;
	u8 hdr[3];
	u8 csum, calc = 0;
	int ret, i;

	if (!len || len > PIC_EEPROM_MAX_LEN)
		return -EINVAL;

	qnap_ts228_pic_wait_idle(pic);
	qnap_ts228_pic_reset_rx(pic);

	ret = qnap_ts228_pic_write(pic, &sync, 1);
	if (ret)
		return ret;

	usleep_range(2000, 3000);
	ret = qnap_ts228_pic_wait_ack(pic);
	if (ret)
		return ret;

	while (qnap_ts228_pic_pop_rx(pic, &stale))
		;

	hdr[0] = PIC_CMD_EEPROM_READ;
	hdr[1] = offset;
	hdr[2] = len;
	ret = qnap_ts228_pic_write(pic, hdr, 3);
	if (ret)
		return ret;

	usleep_range(2000, 3000);
	ret = qnap_ts228_pic_wait_ack(pic);
	if (ret)
		return ret;

	/* Payload may contain arbitrary bytes including 0x40. */
	for (i = 0; i < len; i++) {
		ret = qnap_ts228_pic_wait_byte(pic, &buf[i]);
		if (ret)
			return ret;
		calc = (calc + buf[i]) & 0xff;
	}

	ret = qnap_ts228_pic_wait_byte(pic, &csum);
	if (ret)
		return ret;
	if (csum != calc) {
		dev_dbg(&pic->serdev->dev,
			"eeprom read csum mismatch off=0x%02x got=0x%02x want=0x%02x\n",
			offset, csum, calc);
		return -EIO;
	}

	pic->fan_holdoff_until = jiffies + msecs_to_jiffies(1000);
	return 0;
}

static int qnap_ts228_pic_eeprom_write(struct qnap_ts228_pic *pic, u8 offset,
				       const u8 *buf, u8 len)
{
	u8 stale;
	u8 sync = PIC_CMD_SYNC;
	u8 hdr[3];
	u8 csum = 0;
	int ret, i;

	if (!len || len > PIC_EEPROM_MAX_LEN)
		return -EINVAL;

	for (i = 0; i < len; i++)
		csum = (csum + buf[i]) & 0xff;

	qnap_ts228_pic_wait_idle(pic);
	qnap_ts228_pic_reset_rx(pic);

	ret = qnap_ts228_pic_write(pic, &sync, 1);
	if (ret)
		return ret;

	usleep_range(2000, 3000);
	ret = qnap_ts228_pic_wait_ack(pic);
	if (ret)
		return ret;

	while (qnap_ts228_pic_pop_rx(pic, &stale))
		;

	hdr[0] = PIC_CMD_EEPROM_WRITE;
	hdr[1] = offset;
	hdr[2] = len;
	ret = qnap_ts228_pic_write(pic, hdr, 3);
	if (ret)
		return ret;

	usleep_range(2000, 3000);
	ret = qnap_ts228_pic_wait_ack(pic);
	if (ret)
		return ret;

	for (i = 0; i < len; i++) {
		ret = qnap_ts228_pic_write(pic, &buf[i], 1);
		if (ret)
			return ret;
	}

	ret = qnap_ts228_pic_write(pic, &csum, 1);
	if (ret)
		return ret;

	usleep_range(2000, 3000);
	ret = qnap_ts228_pic_wait_ack(pic);
	if (ret)
		return ret;

	if (!qnap_ts228_pic_wait_rx(pic, 50))
		qnap_ts228_pic_pop_rx(pic, &stale);

	pic->fan_holdoff_until = jiffies + msecs_to_jiffies(2000);
	return 0;
}

/*
 * Fan RPM readback: 0xf6 sync, ack, 0xf7, ack, one data byte (RPM = data * 60).
 * Board-verified reliable (5/5 consistent reads at boot).
 *
 * Temperature readback (0xf6/0xf8) also replies -- ret=0, never times out --
 * but the data byte was 0x00 on every sample taken, including a run polling
 * once a minute out to 11+ minutes of uptime. Confirmed this is not a probe-
 * time/warm-up artifact: the stock QNAP firmware's own hal_app never reads
 * this byte either, despite /etc/model.conf claiming SYSTEM_TEMP_UNIT=PIC --
 * `hal_app --se_sys_get_temp` tracks /sys/class/thermal/thermal_zone0/temp
 * (the SoC's own rtk_thermal zone) in exact lockstep (e.g. 49492 -> 48203
 * millidegrees alongside a reported 49 -> 48), not the PIC. So there is no
 * working system-temperature sensor behind this PIC command on this board;
 * it is intentionally not exposed via hwmon.
 */
static int qnap_ts228_pic_read_fan_rpm(struct qnap_ts228_pic *pic, unsigned int *rpm)
{
	u8 sync = PIC_CMD_SYNC;
	u8 subcmd = 0xf7;
	u8 val;
	int ret;

	qnap_ts228_pic_wait_idle(pic);
	qnap_ts228_pic_reset_rx(pic);

	ret = qnap_ts228_pic_write(pic, &sync, 1);
	if (ret)
		return ret;
	usleep_range(2000, 3000);
	ret = qnap_ts228_pic_wait_ack(pic);
	if (ret)
		return ret;

	ret = qnap_ts228_pic_write(pic, &subcmd, 1);
	if (ret)
		return ret;
	usleep_range(2000, 3000);
	ret = qnap_ts228_pic_wait_ack(pic);
	if (ret)
		return ret;

	ret = qnap_ts228_pic_wait_byte(pic, &val);
	if (ret)
		return ret;

	*rpm = val * 60;
	pic->fan_holdoff_until = jiffies + msecs_to_jiffies(200);
	return 0;
}

static int qnap_ts228_pic_set_power_recovery(struct qnap_ts228_pic *pic,
					     unsigned int mode)
{
	u8 cmd;
	int ret;

	switch (mode) {
	case 0:
	case 2:
		cmd = PIC_CMD_PWR_RECOVERY_A;
		break;
	case 3:
		cmd = PIC_CMD_PWR_RECOVERY_B;
		break;
	default:
		return -EINVAL;
	}

	ret = qnap_ts228_pic_send_cmd(pic, cmd);
	if (ret)
		return ret;

	pic->power_recovery = mode;
	pic->fan_holdoff_until = jiffies + msecs_to_jiffies(1000);
	return 0;
}

/* mode: 0 = short beep (0x50), 1 = long beep (0x51) */
static int qnap_ts228_pic_buzzer(struct qnap_ts228_pic *pic, unsigned int mode)
{
	u8 cmd;
	int ret;

	switch (mode) {
	case 0:
		cmd = PIC_CMD_BUZZER_SHORT;
		break;
	case 1:
		cmd = PIC_CMD_BUZZER_LONG;
		break;
	default:
		return -EINVAL;
	}

	ret = qnap_ts228_pic_send_cmd(pic, cmd);
	if (ret)
		return ret;

	pic->fan_holdoff_until = jiffies + msecs_to_jiffies(mode ? 2000 : 500);
	return 0;
}

/*
 * PIC-level LED states:
 *   status (one bi-color LED): 0=off, 1=green, 2=red, 3=blink
 *   usb: 0=off, 1=on, 2=blink
 */
static int qnap_ts228_pic_led_apply(struct qnap_ts228_pic *pic,
				    enum qnap_ts228_led_id id, u8 state)
{
	int ret = 0;

	switch (id) {
	case QNAP_LED_STATUS:
		if (state > QNAP_STATUS_BLINK)
			return -EINVAL;
		/* Stock: 0x59 then mode. Gap gives PIC time before next opcode. */
		ret = qnap_ts228_pic_send_cmd(pic, PIC_CMD_STATUS_OFF);
		if (ret)
			return ret;
		if (state == QNAP_STATUS_OFF)
			break;
		usleep_range(100000, 150000);
		if (state == QNAP_STATUS_GREEN)
			ret = qnap_ts228_pic_send_cmd(pic, PIC_CMD_STATUS_GREEN);
		else if (state == QNAP_STATUS_RED)
			ret = qnap_ts228_pic_send_cmd(pic, PIC_CMD_STATUS_RED);
		else
			ret = qnap_ts228_pic_send_cmd(pic, PIC_CMD_STATUS_BLINK);
		break;
	case QNAP_LED_USB:
		if (state > 2)
			return -EINVAL;
		if (state == 0)
			ret = qnap_ts228_pic_send_cmd(pic, PIC_CMD_USB_OFF);
		else if (state == 2)
			ret = qnap_ts228_pic_send_cmd(pic, PIC_CMD_USB_BLINK);
		else
			ret = qnap_ts228_pic_send_cmd(pic, PIC_CMD_USB_ON);
		break;
	default:
		return -EINVAL;
	}

	/* Keep monitor reads off the bus briefly after LED traffic. */
	if (!ret)
		pic->fan_holdoff_until = jiffies + msecs_to_jiffies(2000);

	return ret;
}

/*
 * Status LED: exposed as two standard led_classdevs ("red:status",
 * "green:status"), matching the naming/shape mainline's qnap-mcu-leds.c uses
 * for the same kind of bi-color indicator, even though this PIC's status LED
 * is a single 4-state device (off/green/red/blink) rather than two
 * independently blinkable channels. Both colours "on" at once, or a blink
 * request on only one of them, has no direct hardware state -- fall back to
 * the shared blink command for those cases, same as mainline's own fallback
 * for its otherwise-unencodable combinations.
 */
static int qnap_ts228_status_recompute(struct qnap_ts228_pic *pic)
{
	u8 state;

	if (pic->status_red_blink || pic->status_green_blink)
		state = QNAP_STATUS_BLINK;
	else if (pic->status_red_on && pic->status_green_on)
		state = QNAP_STATUS_BLINK;
	else if (pic->status_red_on)
		state = QNAP_STATUS_RED;
	else if (pic->status_green_on)
		state = QNAP_STATUS_GREEN;
	else
		state = QNAP_STATUS_OFF;

	return qnap_ts228_pic_led_apply(pic, QNAP_LED_STATUS, state);
}

static int qnap_ts228_status_led_set_locked(struct qnap_ts228_pic *pic,
					    bool is_red, enum led_brightness brightness)
{
	if (is_red) {
		pic->status_red_on = brightness != 0;
		if (!brightness)
			pic->status_red_blink = false;
	} else {
		pic->status_green_on = brightness != 0;
		if (!brightness)
			pic->status_green_blink = false;
	}

	return qnap_ts228_status_recompute(pic);
}

static int qnap_ts228_status_red_led_set(struct led_classdev *led_cdev,
					 enum led_brightness brightness)
{
	struct qnap_ts228_pic *pic = container_of(led_cdev, struct qnap_ts228_pic,
						  led_status_red);

	guard(mutex)(&pic->lock);
	return qnap_ts228_status_led_set_locked(pic, true, brightness);
}

static int qnap_ts228_status_green_led_set(struct led_classdev *led_cdev,
					   enum led_brightness brightness)
{
	struct qnap_ts228_pic *pic = container_of(led_cdev, struct qnap_ts228_pic,
						  led_status_green);

	guard(mutex)(&pic->lock);
	return qnap_ts228_status_led_set_locked(pic, false, brightness);
}

static int qnap_ts228_status_led_blink_set_locked(struct qnap_ts228_pic *pic,
						  bool is_red,
						    unsigned long *delay_on,
						    unsigned long *delay_off)
{
	/* Single hardware-driven blink rate; report a representative value. */
	*delay_on = 500;
	*delay_off = 500;

	if (is_red) {
		pic->status_red_on = true;
		pic->status_red_blink = true;
	} else {
		pic->status_green_on = true;
		pic->status_green_blink = true;
	}

	return qnap_ts228_status_recompute(pic);
}

static int qnap_ts228_status_red_led_blink_set(struct led_classdev *led_cdev,
					       unsigned long *delay_on,
					       unsigned long *delay_off)
{
	struct qnap_ts228_pic *pic = container_of(led_cdev, struct qnap_ts228_pic,
						  led_status_red);

	guard(mutex)(&pic->lock);
	return qnap_ts228_status_led_blink_set_locked(pic, true, delay_on, delay_off);
}

static int qnap_ts228_status_green_led_blink_set(struct led_classdev *led_cdev,
						 unsigned long *delay_on,
						  unsigned long *delay_off)
{
	struct qnap_ts228_pic *pic = container_of(led_cdev, struct qnap_ts228_pic,
						  led_status_green);

	guard(mutex)(&pic->lock);
	return qnap_ts228_status_led_blink_set_locked(pic, false, delay_on, delay_off);
}

static int qnap_ts228_pic_register_status_leds(struct device *dev,
					       struct qnap_ts228_pic *pic)
{
	int ret;

	pic->led_status_red.name = "red:status";
	pic->led_status_red.brightness_set_blocking = qnap_ts228_status_red_led_set;
	pic->led_status_red.blink_set = qnap_ts228_status_red_led_blink_set;
	pic->led_status_red.brightness = 0;
	pic->led_status_red.max_brightness = 1;

	pic->led_status_green.name = "green:status";
	pic->led_status_green.brightness_set_blocking = qnap_ts228_status_green_led_set;
	pic->led_status_green.blink_set = qnap_ts228_status_green_led_blink_set;
	pic->led_status_green.brightness = 0;
	pic->led_status_green.max_brightness = 1;

	ret = devm_led_classdev_register(dev, &pic->led_status_red);
	if (ret)
		return ret;

	ret = devm_led_classdev_register(dev, &pic->led_status_green);
	if (ret)
		return ret;

	guard(mutex)(&pic->lock);
	return qnap_ts228_pic_led_apply(pic, QNAP_LED_STATUS, QNAP_STATUS_OFF);
}

/* USB LED: direct off/on/blink match, one led_classdev. */
static int qnap_ts228_usb_led_set(struct led_classdev *led_cdev,
				  enum led_brightness brightness)
{
	struct qnap_ts228_pic *pic = container_of(led_cdev, struct qnap_ts228_pic, led_usb);

	guard(mutex)(&pic->lock);
	return qnap_ts228_pic_led_apply(pic, QNAP_LED_USB, brightness ? 1 : 0);
}

static int qnap_ts228_usb_led_blink_set(struct led_classdev *led_cdev,
					unsigned long *delay_on,
					unsigned long *delay_off)
{
	struct qnap_ts228_pic *pic = container_of(led_cdev, struct qnap_ts228_pic, led_usb);

	*delay_on = 500;
	*delay_off = 500;

	guard(mutex)(&pic->lock);
	return qnap_ts228_pic_led_apply(pic, QNAP_LED_USB, 2);
}

static int qnap_ts228_pic_register_usb_led(struct device *dev,
					   struct qnap_ts228_pic *pic)
{
	int ret;

	pic->led_usb.name = "usb:blue:disk";
	pic->led_usb.brightness_set_blocking = qnap_ts228_usb_led_set;
	pic->led_usb.blink_set = qnap_ts228_usb_led_blink_set;
	pic->led_usb.brightness = 0;
	pic->led_usb.max_brightness = 1;

	ret = devm_led_classdev_register(dev, &pic->led_usb);
	if (ret)
		return ret;

	guard(mutex)(&pic->lock);
	return qnap_ts228_pic_led_apply(pic, QNAP_LED_USB, 0);
}

/* Trim trailing NUL/space from PIC EEPROM ASCII blobs. */
static size_t qnap_ts228_pic_ascii_len(const u8 *buf, size_t len)
{
	while (len && (buf[len - 1] == '\0' || buf[len - 1] == ' '))
		len--;
	return len;
}

static int qnap_ts228_pic_mac_ascii_to_bin(const u8 *raw, size_t raw_len,
					   u8 mac[ETH_ALEN])
{
	char tmp[PIC_EEPROM_MAC_LEN + 1];
	size_t len;

	len = qnap_ts228_pic_ascii_len(raw, raw_len);
	if (!len || len >= sizeof(tmp))
		return -EINVAL;

	memcpy(tmp, raw, len);
	tmp[len] = '\0';

	if (!mac_pton(tmp, mac))
		return -EINVAL;

	return 0;
}

/*
 * Must be called with pic->lock held.
 * Loads MAC from PIC EEPROM and populates pic->mac_addr cache.
 */
static int qnap_ts228_pic_load_mac_locked(struct qnap_ts228_pic *pic)
{
	u8 raw[PIC_EEPROM_MAC_LEN];
	int ret;

	if (pic->mac_valid)
		return 0;

	ret = qnap_ts228_pic_eeprom_read(pic, PIC_EEPROM_MAC_OFF, raw,
					 PIC_EEPROM_MAC_LEN);
	if (ret)
		return ret;

	ret = qnap_ts228_pic_mac_ascii_to_bin(raw, PIC_EEPROM_MAC_LEN,
					      pic->mac_addr);
	if (ret)
		return ret;

	pic->mac_valid = true;
	return 0;
}

static int qnap_ts228_pic_nvmem_mac_reg_read(void *priv, unsigned int offset,
					     void *val, size_t bytes)
{
	struct qnap_ts228_pic *pic = priv;
	u8 *out = val;
	int ret;

	if (!out)
		return -EINVAL;
	if (offset >= ETH_ALEN || offset + bytes > ETH_ALEN)
		return -EINVAL;

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_load_mac_locked(pic);
	if (ret)
		return ret;

	memcpy(out, pic->mac_addr + offset, bytes);
	return 0;
}

static ssize_t serial_number_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);
	u8 raw[PIC_EEPROM_SN_LEN];
	const char *sn;
	size_t len;
	int ret;

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_eeprom_read(pic, PIC_EEPROM_SN_OFF, raw,
					 PIC_EEPROM_SN_LEN);
	if (ret)
		return ret;

	len = qnap_ts228_pic_ascii_len(raw, PIC_EEPROM_SN_LEN);
	sn = (const char *)raw;
	if (len >= 3 && !strncmp(sn, "SN:", 3)) {
		sn += 3;
		len -= 3;
	}

	return sysfs_emit(buf, "%.*s\n", (int)len, sn);
}

static ssize_t serial_number_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);
	u8 raw[PIC_EEPROM_SN_LEN];
	char tmp[PIC_EEPROM_SN_LEN + 1];
	const char *in = buf;
	size_t in_len = count;
	int ret;

	while (in_len && (in[in_len - 1] == '\n' || in[in_len - 1] == '\r'))
		in_len--;
	if (in_len >= 3 && !strncmp(in, "SN:", 3)) {
		in += 3;
		in_len -= 3;
	}
	if (!in_len || in_len > PIC_EEPROM_SN_LEN - 3)
		return -EINVAL;

	memcpy(tmp, in, in_len);
	tmp[in_len] = '\0';

	memset(raw, 0, sizeof(raw));
	raw[0] = 'S';
	raw[1] = 'N';
	raw[2] = ':';
	memcpy(raw + 3, tmp, in_len);

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_eeprom_write(pic, PIC_EEPROM_SN_OFF, raw,
					  PIC_EEPROM_SN_LEN);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(serial_number);

static ssize_t mac_address_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);
	u8 raw[PIC_EEPROM_MAC_LEN];
	size_t len;
	int ret;

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_eeprom_read(pic, PIC_EEPROM_MAC_OFF, raw,
					 PIC_EEPROM_MAC_LEN);
	if (ret)
		return ret;

	len = qnap_ts228_pic_ascii_len(raw, PIC_EEPROM_MAC_LEN);
	return sysfs_emit(buf, "%.*s\n", (int)len, raw);
}

static ssize_t mac_address_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);
	u8 raw[PIC_EEPROM_MAC_LEN];
	u8 mac[ETH_ALEN];
	char tmp[32];
	size_t in_len = count;
	int ret;

	while (in_len && (buf[in_len - 1] == '\n' || buf[in_len - 1] == '\r'))
		in_len--;
	if (!in_len || in_len >= sizeof(tmp))
		return -EINVAL;

	memcpy(tmp, buf, in_len);
	tmp[in_len] = '\0';

	if (!mac_pton(tmp, mac))
		return -EINVAL;

	/* Stock stores colon-separated ASCII + trailing NUL (18 bytes). */
	memset(raw, 0, sizeof(raw));
	snprintf((char *)raw, sizeof(raw), "%02X:%02X:%02X:%02X:%02X:%02X",
		 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_eeprom_write(pic, PIC_EEPROM_MAC_OFF, raw,
					  PIC_EEPROM_MAC_LEN);
	if (!ret) {
		memcpy(pic->mac_addr, mac, ETH_ALEN);
		pic->mac_valid = true;
	}

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(mac_address);

static ssize_t power_recovery_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);

	guard(mutex)(&pic->lock);
	/* PIC has no get; report last mode written via this driver. */
	return sysfs_emit(buf, "%u\n", pic->power_recovery);
}

static ssize_t power_recovery_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);
	unsigned int mode;
	int ret;

	ret = kstrtouint(buf, 0, &mode);
	if (ret)
		return ret;

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_set_power_recovery(pic, mode);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(power_recovery);

static struct attribute *qnap_ts228_pic_attrs[] = {
	&dev_attr_serial_number.attr,
	&dev_attr_mac_address.attr,
	&dev_attr_power_recovery.attr,
	NULL,
};
ATTRIBUTE_GROUPS(qnap_ts228_pic);

/*
 * Buzzer: exposed as EV_SND on a plain input device, matching mainline's
 * qnap-mcu-input.c rather than a bespoke sysfs attribute -- same userspace
 * ABI as the TS-233/TS-433 driver family (SND_BELL/SND_TONE, beep runtime
 * fixed by the PIC, "value" is on/off only). No KEY_POWER capability here:
 * unlike those MCUs, this PIC's power button produces no event of any kind
 * on this board (board-verified -- see the driver's own power-button note
 * and project memory), so there is nothing to poll.
 */
static void qnap_ts228_pic_beep_work(struct work_struct *work)
{
	struct qnap_ts228_pic *pic = container_of(work, struct qnap_ts228_pic, beep_work);
	unsigned int mode = pic->beep_type == SND_TONE ? 1 : 0;

	guard(mutex)(&pic->lock);
	qnap_ts228_pic_buzzer(pic, mode);
}

static int qnap_ts228_pic_input_event(struct input_dev *input, unsigned int type,
				      unsigned int code, int value)
{
	struct qnap_ts228_pic *pic = input_get_drvdata(input);

	if (type != EV_SND || (code != SND_BELL && code != SND_TONE))
		return -EOPNOTSUPP;

	if (value < 0)
		return -EINVAL;

	/* beep runtime is determined by the PIC */
	if (value == 0)
		return 0;

	pic->beep_type = code;
	schedule_work(&pic->beep_work);

	return 0;
}

static void qnap_ts228_pic_input_close(struct input_dev *input)
{
	struct qnap_ts228_pic *pic = input_get_drvdata(input);

	cancel_work_sync(&pic->beep_work);
}

static int qnap_ts228_pic_register_beeper(struct device *dev, struct qnap_ts228_pic *pic)
{
	struct input_dev *input;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	pic->input = input;
	input_set_drvdata(input, pic);

	input->name = "qnap-ts228-pic";
	input->phys = "qnap-ts228-pic/input0";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	input->event = qnap_ts228_pic_input_event;
	input->close = qnap_ts228_pic_input_close;

	input_set_capability(input, EV_SND, SND_BELL);
	input_set_capability(input, EV_SND, SND_TONE);

	INIT_WORK(&pic->beep_work, qnap_ts228_pic_beep_work);

	return input_register_device(input);
}

static unsigned int qnap_ts228_pic_pwm_to_level(struct qnap_ts228_pic *pic, unsigned int pwm)
{
	unsigned int i, best = 0;

	if (!pic->fan_cooling_levels || !pic->fan_max_state)
		return clamp(pwm, 0U, PIC_CMD_FAN_LEVELS - 1);

	if (pwm == 0)
		return 0;

	for (i = 1; i <= pic->fan_max_state; i++) {
		if (pwm >= pic->fan_cooling_levels[i])
			best = i;
	}

	return min(best, PIC_CMD_FAN_LEVELS - 1);
}

static int qnap_ts228_pic_set_pwm(struct qnap_ts228_pic *pic, unsigned int pwm)
{
	unsigned int level = qnap_ts228_pic_pwm_to_level(pic, pwm);
	int ret;

	ret = qnap_ts228_pic_set_fan_level(pic, level);
	if (ret)
		return ret;

	pic->fan_level = level;
	pic->fan_state = level;
	return 0;
}

static int qnap_ts228_pic_hwmon_read(struct device *dev,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel, long *val)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);

	guard(mutex)(&pic->lock);

	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			if (pic->fan_cooling_levels)
				*val = pic->fan_cooling_levels[pic->fan_level];
			else
				*val = pic->fan_level * 64;
			return 0;
		case hwmon_pwm_enable:
			*val = pic->pwm_auto ? QNAP_PIC_PWM_AUTO : QNAP_PIC_PWM_MANUAL;
			return 0;
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_fan: {
		unsigned int rpm;
		int ret;

		ret = qnap_ts228_pic_read_fan_rpm(pic, &rpm);
		if (ret)
			return ret;
		*val = rpm;
		return 0;
	}
	default:
		return -EOPNOTSUPP;
	}
}

static int qnap_ts228_pic_hwmon_write(struct device *dev,
				      enum hwmon_sensor_types type,
				      u32 attr, int channel, long val)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;

	guard(mutex)(&pic->lock);

	switch (attr) {
	case hwmon_pwm_enable:
		if (val == QNAP_PIC_PWM_AUTO) {
			pic->pwm_auto = true;
			return 0;
		}
		if (val == QNAP_PIC_PWM_MANUAL) {
			pic->pwm_auto = false;
			return 0;
		}
		if (val == QNAP_PIC_PWM_OFF)
			return qnap_ts228_pic_set_pwm(pic, 0);
		return -EINVAL;
	case hwmon_pwm_input:
		if (val < 0 || val > 255)
			return -EINVAL;
		pic->pwm_auto = false;
		return qnap_ts228_pic_set_pwm(pic, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t qnap_ts228_pic_hwmon_is_visible(const void *data,
					       enum hwmon_sensor_types type,
					       u32 attr, int channel)
{
	switch (type) {
	case hwmon_pwm:
		return 0644;
	case hwmon_fan:
		return 0444;
	default:
		return 0;
	}
}

static const struct hwmon_ops qnap_ts228_pic_hwmon_ops = {
	.is_visible = qnap_ts228_pic_hwmon_is_visible,
	.read = qnap_ts228_pic_hwmon_read,
	.write = qnap_ts228_pic_hwmon_write,
};

static int qnap_ts228_pic_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct qnap_ts228_pic *pic = cdev->devdata;

	*state = pic->fan_max_state;
	return 0;
}

static int qnap_ts228_pic_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct qnap_ts228_pic *pic = cdev->devdata;

	*state = pic->fan_state;
	return 0;
}

static int qnap_ts228_pic_set_cur_state(struct thermal_cooling_device *cdev,
					unsigned long state)
{
	struct qnap_ts228_pic *pic = cdev->devdata;
	unsigned int pwm;
	int ret;

	if (state > pic->fan_max_state)
		return -EINVAL;
	if (!pic->pwm_auto)
		return 0;
	if (state == pic->fan_state)
		return 0;

	pwm = pic->fan_cooling_levels ? pic->fan_cooling_levels[state] : state * 51;

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_set_pwm(pic, pwm);
	return ret;
}

static const struct thermal_cooling_device_ops qnap_ts228_pic_cooling_ops = {
	.get_max_state = qnap_ts228_pic_get_max_state,
	.get_cur_state = qnap_ts228_pic_get_cur_state,
	.set_cur_state = qnap_ts228_pic_set_cur_state,
};

static void qnap_ts228_pic_fan_node_release(void *data)
{
	struct qnap_ts228_pic *pic = data;

	fwnode_handle_put(pic->fan_node);
}

static int qnap_ts228_pic_parse_cooling(struct device *dev,
					struct qnap_ts228_pic *pic)
{
	struct fwnode_handle *fwnode;
	int num, i, ret;

	fwnode = device_get_named_child_node(dev, "fan-0");
	if (!fwnode)
		return 0;

	pic->fan_node = fwnode;
	ret = devm_add_action_or_reset(dev, qnap_ts228_pic_fan_node_release, pic);
	if (ret)
		return ret;

	num = fwnode_property_count_u32(fwnode, "cooling-levels");
	if (num <= 0)
		return dev_err_probe(dev, num ? : -EINVAL,
				     "Failed to count cooling-levels\n");

	pic->fan_cooling_levels = devm_kcalloc(dev, num, sizeof(u32), GFP_KERNEL);
	if (!pic->fan_cooling_levels)
		return -ENOMEM;

	ret = fwnode_property_read_u32_array(fwnode, "cooling-levels",
					     pic->fan_cooling_levels, num);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read cooling-levels\n");

	for (i = 0; i < num; i++) {
		if (pic->fan_cooling_levels[i] > 255)
			return dev_err_probe(dev, -EINVAL,
					     "cooling-levels[%d] out of range\n", i);
	}

	pic->fan_max_state = num - 1;
	return 0;
}

static const struct hwmon_channel_info * const qnap_ts228_pic_channels[] = {
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT),
	NULL
};

static int qnap_ts228_pic_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct qnap_ts228_pic *pic;
	int ret;

	pic = devm_kzalloc(dev, sizeof(*pic), GFP_KERNEL);
	if (!pic)
		return -ENOMEM;

	pic->serdev = serdev;
	/*
	 * jiffies starts at INITIAL_JIFFIES, five minutes before it wraps,
	 * precisely so that a timeout left at zero shows up. Leaving this at
	 * zero makes time_before(jiffies, 0) true for the first 300 seconds
	 * after boot, and the monitor thread skips every cycle until then.
	 */
	pic->fan_holdoff_until = jiffies;
	spin_lock_init(&pic->rx_lock);
	init_completion(&pic->rx_done);
	mutex_init(&pic->lock);
	serdev_device_set_drvdata(serdev, pic);

	serdev_device_set_client_ops(serdev, &qnap_ts228_pic_serdev_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(serdev, QNAP_TS228_PIC_BAUDRATE);
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to set parity\n");

	ret = serdev_device_set_tiocm(serdev, TIOCM_RTS | TIOCM_DTR, 0);
	if (ret)
		dev_warn(dev, "Failed to assert RTS/DTR: %d\n", ret);

	ret = qnap_ts228_pic_parse_cooling(dev, pic);
	if (ret)
		return ret;

	mutex_lock(&pic->lock);
	ret = qnap_ts228_pic_start_monitor(pic);
	if (!ret) {
		pic->fan_max_level = PIC_CMD_FAN_LEVELS - 1;
		if (!pic->fan_cooling_levels)
			pic->fan_max_state = pic->fan_max_level;
		pic->fan_level = 0;
		pic->fan_state = 0;
		pic->pwm_auto = true;
	}
	mutex_unlock(&pic->lock);

	if (ret)
		return dev_err_probe(dev, ret, "PIC init failed\n");

	pic->info.ops = &qnap_ts228_pic_hwmon_ops;
	pic->info.info = qnap_ts228_pic_channels;

	pic->hwmon_dev = devm_hwmon_device_register_with_info(dev, "qnapts228",
							      pic, &pic->info, NULL);
	if (IS_ERR(pic->hwmon_dev))
		return dev_err_probe(dev, PTR_ERR(pic->hwmon_dev),
				     "Failed to register hwmon device\n");

	ret = qnap_ts228_pic_register_status_leds(dev, pic);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register status LEDs\n");

	ret = qnap_ts228_pic_register_usb_led(dev, pic);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register USB LED\n");

	ret = qnap_ts228_pic_register_beeper(dev, pic);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register beeper input device\n");

#if IS_ENABLED(CONFIG_NVMEM)
	/*
	 * No MAC preload here at probe time: an EEPROM read this early was
	 * previously observed to time out (board-tested once at the time this
	 * comment was written). Reads issued later, once user space is up
	 * (e.g. this driver's own mac_address/serial_number attributes), have
	 * since been confirmed to work reliably, so that earlier failure looks
	 * like a probe-time-only PIC readiness window rather than a permanent
	 * limitation. Leaving the preload out avoids spending ~1.5s on a
	 * transfer that may still fail this early and warning about it on
	 * every boot; the cell stays registered and a reader gets an honest
	 * error if the PIC genuinely isn't ready yet.
	 */
	{
		struct nvmem_config cfg = {
			.dev = dev,
			.name = "qnap-ts228-pic-mac",
			.owner = THIS_MODULE,
			.read_only = true,
			.reg_read = qnap_ts228_pic_nvmem_mac_reg_read,
			.size = ETH_ALEN,
			.word_size = 1,
			.stride = 1,
			.priv = pic,
			/* mac-address@0 under the pic DT node */
			.add_legacy_fixed_of_cells = true,
		};
		struct nvmem_device *nvmem;

		nvmem = devm_nvmem_register(dev, &cfg);
		if (IS_ERR(nvmem))
			dev_warn(dev, "Failed to register nvmem MAC cell: %ld\n",
				 PTR_ERR(nvmem));
	}
#endif

	/*
	 * Register the cooling device only after releasing pic->lock.
	 * thermal_of_cooling_device_register() updates the thermal zone and
	 * calls ->set_cur_state(), which would deadlock if probe still held
	 * the mutex.
	 */
	if (IS_ENABLED(CONFIG_THERMAL) && pic->fan_cooling_levels) {
		pic->cdev = devm_thermal_of_cooling_device_register(dev,
								    to_of_node(pic->fan_node),
					"qnap-ts228-pic", pic,
					&qnap_ts228_pic_cooling_ops);
		if (IS_ERR(pic->cdev))
			return dev_err_probe(dev, PTR_ERR(pic->cdev),
					     "Failed to register cooling device\n");
	}

	pic->reboot_nb.notifier_call = qnap_ts228_pic_reboot_notify;
	pic->reboot_nb.priority = 128;
	ret = register_reboot_notifier(&pic->reboot_nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register reboot notifier\n");

	ret = devm_add_action_or_reset(dev, qnap_ts228_pic_unregister_reboot,
				       pic);
	if (ret)
		return ret;

	dev_info(dev, "QNAP TS-228 PIC initialized\n");
	return 0;
}

static const struct of_device_id qnap_ts228_pic_of_match[] = {
	{ .compatible = "qnap,ts228-pic" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, qnap_ts228_pic_of_match);

static struct serdev_device_driver qnap_ts228_pic_driver = {
	.probe = qnap_ts228_pic_probe,
	.shutdown = qnap_ts228_pic_shutdown,
	.driver = {
		.name = "qnap-ts228-pic",
		.of_match_table = qnap_ts228_pic_of_match,
		.dev_groups = qnap_ts228_pic_groups,
	},
};
module_serdev_device_driver(qnap_ts228_pic_driver);

MODULE_AUTHOR("Stephan <linux-qnap-ts228>");
MODULE_DESCRIPTION("QNAP TS-228 PIC hwmon and LED driver");
MODULE_LICENSE("GPL");
