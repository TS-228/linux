// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hwmon + LED driver for the QNAP TS-228 front-panel PIC on UART1.
 *
 * Protocol reverse-engineered from QTS 3.10 libuLinux_hal.so (pic_sys_*):
 *  - 19200 8N1 raw serial (stock tcsetattr c_cflag B19200|CS8|CREAD|CLOCAL)
 *  - Init: write 0xf2, drain RX (~200 ms). Stock pic_handle_cmd(0xa1) is an
 *    internal monitor opcode that sends 0xf6 (sync read), not UART byte 0xa1.
 *  - Fan set: single byte 0x30..0x34 (pic_sys_set_fan_speed table; mode 5 = 0x34)
 *  - Fan RPM read: 0xf6, ack, 0xf7, ack, data byte; rpm = byte * 60
 *  - Temp read: same with 0xf8; temp in degrees C is the data byte
 *  - LEDs (se_sys_set_*_led PIC backend; status is one bi-color LED):
 *      status: off 0x59; green 0x56; red 0x57; blink 0x58
 *      USB on 0x60; blink 0x61; off 0x62
 *  - EEPROM: 0xf6 sync, then 0xa1/0xa0 + offset + len (+ data + csum for write)
 *      MAC ASCII at 0x10; board SN ("SN:…") at 0x38 (24 bytes)
 *  - Power recovery: 0x48 (modes 0/2), 0x49 (mode 3)
 *  - Buzzer (vendor include/qnap/pic.h): short 0x50, long 0x51
 *  - Software shutdown: 0x41 (QNAP_PIC_SOFTWARE_SHUTDOWN); cuts PSU
 *
 * Soft KEY_POWER is not available: the front power button does not send a
 * UART event on TS-228 (stock uses se_pwb / eMCU-SIO; long-press is PIC HW).
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/hwmon.h>
#include <linux/kthread.h>
#include <linux/serial.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/thermal.h>

static bool debug_rx;
module_param(debug_rx, bool, 0644);
MODULE_PARM_DESC(debug_rx, "Log every PIC RX byte at info level");

#define QNAP_TS228_PIC_BAUDRATE		19200
#define QNAP_TS228_PIC_TIMEOUT_MS	1000
#define QNAP_TS228_PIC_RX_SIZE		256

#define PIC_CMD_INIT			0xf2
#define PIC_CMD_SYNC			0xf6
#define PIC_CMD_ACK			0xaa
#define PIC_CMD_READ_FAN		0xf7
#define PIC_CMD_READ_TEMP		0xf8
#define PIC_CMD_FAN_LEVELS		5
#define PIC_MONITOR_INTERVAL_MS		6000
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
	QNAP_LED_COUNT,
};

struct qnap_ts228_pic {
	struct serdev_device *serdev;
	struct device *hwmon_dev;
	struct mutex lock;

	u8 rx_buf[QNAP_TS228_PIC_RX_SIZE];
	unsigned int rx_len;
	struct completion rx_done;

	unsigned int fan_level;
	unsigned int fan_max_level;

	int fan_rpm;
	int temp_millic;
	bool fan_valid;
	bool temp_valid;

	struct task_struct *monitor_task;

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

	u8 led_state[QNAP_LED_COUNT]; /* status/usb mode values */
	unsigned int power_recovery;  /* last written mode (0/2/3) */
};

static void qnap_ts228_pic_reset_rx(struct qnap_ts228_pic *pic)
{
	pic->rx_len = 0;
	reinit_completion(&pic->rx_done);
}

static size_t qnap_ts228_pic_receive_buf(struct serdev_device *serdev,
					 const u8 *data, size_t count)
{
	struct qnap_ts228_pic *pic = serdev_device_get_drvdata(serdev);
	size_t i;
	bool got_data = false;

	if (debug_rx && count)
		dev_info(&serdev->dev, "pic-rx (%zu): %*ph\n",
			 count, (int)count, data);

	for (i = 0; i < count; i++) {
		u8 b = data[i];

		if (pic->rx_len >= QNAP_TS228_PIC_RX_SIZE)
			continue;

		pic->rx_buf[pic->rx_len++] = b;
		got_data = true;
	}

	if (got_data) {
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
	if (pic->rx_len)
		return 0;

	ret = wait_for_completion_timeout(&pic->rx_done,
					  msecs_to_jiffies(timeout_ms));

	return ret ? 0 : -ETIMEDOUT;
}

static void qnap_ts228_pic_consume_rx(struct qnap_ts228_pic *pic, unsigned int count)
{
	if (count >= pic->rx_len) {
		qnap_ts228_pic_reset_rx(pic);
		return;
	}

	pic->rx_len -= count;
	memmove(pic->rx_buf, pic->rx_buf + count, pic->rx_len);
	if (!pic->rx_len)
		reinit_completion(&pic->rx_done);
}

static int qnap_ts228_pic_wait_idle(struct qnap_ts228_pic *pic)
{
	u8 b;
	unsigned long deadline = jiffies + msecs_to_jiffies(500);

	while (time_before(jiffies, deadline)) {
		int ret;

		if (!pic->rx_len) {
			ret = qnap_ts228_pic_wait_rx(pic, 50);
			if (ret)
				continue;
		}

		b = pic->rx_buf[0];
		if (b == 0xe0) {
			qnap_ts228_pic_consume_rx(pic, 1);
			return 0;
		}
		qnap_ts228_pic_consume_rx(pic, 1);
	}

	return -ETIMEDOUT;
}

static int qnap_ts228_pic_wait_ack(struct qnap_ts228_pic *pic)
{
	u8 b;
	unsigned long deadline = jiffies + msecs_to_jiffies(QNAP_TS228_PIC_TIMEOUT_MS);

	while (time_before(jiffies, deadline)) {
		int ret;

		if (!pic->rx_len) {
			ret = qnap_ts228_pic_wait_rx(pic, 50);
			if (ret)
				continue;
		}

		b = pic->rx_buf[0];
		qnap_ts228_pic_consume_rx(pic, 1);
		if (b == PIC_CMD_ACK || b == 0x00)
			return 0;
		if (b == 0xe0)
			continue;
	}

	return -ETIMEDOUT;
}

static int qnap_ts228_pic_wait_byte(struct qnap_ts228_pic *pic, u8 *val)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(QNAP_TS228_PIC_TIMEOUT_MS);

	while (time_before(jiffies, deadline)) {
		if (!pic->rx_len) {
			if (qnap_ts228_pic_wait_rx(pic, 50))
				continue;
		}

		*val = pic->rx_buf[0];
		qnap_ts228_pic_consume_rx(pic, 1);
		return 0;
	}

	return -ETIMEDOUT;
}

/* Fan/temp path: skip framing noise. EEPROM uses wait_byte (payload may be 0xaa). */
static int qnap_ts228_pic_wait_data(struct qnap_ts228_pic *pic, u8 *val)
{
	u8 b;
	unsigned long deadline = jiffies + msecs_to_jiffies(QNAP_TS228_PIC_TIMEOUT_MS);

	while (time_before(jiffies, deadline)) {
		int ret;

		ret = qnap_ts228_pic_wait_byte(pic, &b);
		if (ret)
			continue;
		if (b == PIC_CMD_ACK || b == PIC_CMD_SYNC || b == 0xe0)
			continue;
		*val = b;
		return 0;
	}

	return -ETIMEDOUT;
}

static int qnap_ts228_pic_drain(struct qnap_ts228_pic *pic, unsigned int timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);

	while (time_before(jiffies, deadline)) {
		if (qnap_ts228_pic_wait_rx(pic, 50))
			break;
	}

	return 0;
}

static int qnap_ts228_pic_start_monitor(struct qnap_ts228_pic *pic)
{
	u8 init = PIC_CMD_INIT;
	int ret;

	if (pic->monitor_ready)
		return 0;

	ret = qnap_ts228_pic_write(pic, &init, 1);
	if (ret)
		return ret;

	usleep_range(200000, 250000);
	qnap_ts228_pic_drain(pic, 200);
	qnap_ts228_pic_reset_rx(pic);

	pic->monitor_ready = true;
	dev_info(&pic->serdev->dev, "PIC initialized (0xf2)\n");
	return 0;
}

static void qnap_ts228_pic_log_rx(struct qnap_ts228_pic *pic, const char *tag)
{
	if (!pic->rx_len)
		return;

	dev_info(&pic->serdev->dev, "%s rx (%u): %*ph\n",
		 tag, pic->rx_len, (int)min_t(unsigned int, pic->rx_len, 16U),
		 pic->rx_buf);
}

static int qnap_ts228_pic_read_cmd(struct qnap_ts228_pic *pic, u8 cmd, u8 *val)
{
	u8 sync = PIC_CMD_SYNC;
	int ret, attempt;

	for (attempt = 0; attempt < PIC_READ_RETRIES; attempt++) {
		qnap_ts228_pic_wait_idle(pic);

		ret = qnap_ts228_pic_write(pic, &sync, 1);
		if (ret)
			continue;

		/* Stock waits for ack immediately after 0xf6 (1s timeout), not a fixed sleep.
		 * The 2s usleep is only used by the internal 0xa1 monitor opcode path.
		 */
		usleep_range(2000, 3000);

		ret = qnap_ts228_pic_wait_ack(pic);
		if (ret)
			continue;

		ret = qnap_ts228_pic_write(pic, &cmd, 1);
		if (ret)
			continue;

		usleep_range(2000, 3000);

		ret = qnap_ts228_pic_wait_ack(pic);
		if (ret)
			continue;

		ret = qnap_ts228_pic_wait_data(pic, val);
		if (!ret)
			return 0;
	}

	dev_dbg(&pic->serdev->dev, "read 0x%02x failed after %d tries\n",
		cmd, PIC_READ_RETRIES);
	return -ETIMEDOUT;
}

static int qnap_ts228_pic_monitor(void *data)
{
	struct qnap_ts228_pic *pic = data;
	u8 raw;

	while (!kthread_should_stop()) {
		mutex_lock(&pic->lock);
		if (time_before(jiffies, pic->fan_holdoff_until))
			goto monitor_sleep;

		if (!qnap_ts228_pic_read_cmd(pic, PIC_CMD_READ_FAN, &raw)) {
			pic->fan_rpm = raw * 60;
			pic->fan_valid = true;
		}
		if (!qnap_ts228_pic_read_cmd(pic, PIC_CMD_READ_TEMP, &raw)) {
			pic->temp_millic = raw * 1000;
			pic->temp_valid = true;
		}

monitor_sleep:
		mutex_unlock(&pic->lock);

		if (msleep_interruptible(PIC_MONITOR_INTERVAL_MS))
			break;
	}

	return 0;
}

static void qnap_ts228_pic_stop_monitor(void *data)
{
	struct qnap_ts228_pic *pic = data;

	if (pic->monitor_task)
		kthread_stop(pic->monitor_task);
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
	qnap_ts228_pic_log_rx(pic, "fan set");
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

static void qnap_ts228_pic_shutdown(struct device *dev)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct qnap_ts228_pic *pic = serdev_device_get_drvdata(serdev);

	if (!pic)
		return;

	mutex_lock(&pic->lock);
	qnap_ts228_pic_send_cmd(pic, PIC_CMD_SOFTWARE_SHUTDOWN);
	mutex_unlock(&pic->lock);
	dev_emerg(dev, "PIC software shutdown (0x41) via device shutdown\n");
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

	while (pic->rx_len)
		qnap_ts228_pic_consume_rx(pic, 1);

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

	while (pic->rx_len)
		qnap_ts228_pic_consume_rx(pic, 1);

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
		qnap_ts228_pic_consume_rx(pic, 1);

	pic->fan_holdoff_until = jiffies + msecs_to_jiffies(2000);
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
 * LED sysfs values:
 *   led_status: 0=off, 1=green, 2=red, 3=blink  (one bi-color LED)
 *   led_usb:    0=off, 1=on, 2=blink
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

	if (!ret) {
		pic->led_state[id] = state;
		/* Keep monitor reads off the bus briefly after LED traffic. */
		pic->fan_holdoff_until = jiffies + msecs_to_jiffies(2000);
	}

	return ret;
}

static ssize_t qnap_ts228_led_show(struct device *dev,
				   struct device_attribute *attr, char *buf,
				   enum qnap_ts228_led_id id)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);

	guard(mutex)(&pic->lock);
	return sysfs_emit(buf, "%u\n", pic->led_state[id]);
}

static ssize_t qnap_ts228_led_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count,
				    enum qnap_ts228_led_id id)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);
	unsigned int state;
	int ret;

	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_led_apply(pic, id, state);
	return ret ? ret : count;
}

#define QNAP_TS228_LED_ATTR(name, id)					\
static ssize_t name##_show(struct device *dev,				\
			   struct device_attribute *attr, char *buf)	\
{									\
	return qnap_ts228_led_show(dev, attr, buf, id);			\
}									\
static ssize_t name##_store(struct device *dev,				\
			    struct device_attribute *attr,		\
			    const char *buf, size_t count)		\
{									\
	return qnap_ts228_led_store(dev, attr, buf, count, id);		\
}									\
static DEVICE_ATTR_RW(name)

QNAP_TS228_LED_ATTR(led_status, QNAP_LED_STATUS);
QNAP_TS228_LED_ATTR(led_usb, QNAP_LED_USB);

/* Trim trailing NUL/space from PIC EEPROM ASCII blobs. */
static size_t qnap_ts228_pic_ascii_len(const u8 *buf, size_t len)
{
	while (len && (buf[len - 1] == '\0' || buf[len - 1] == ' '))
		len--;
	return len;
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

static ssize_t buzzer_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct qnap_ts228_pic *pic = dev_get_drvdata(dev);
	unsigned int mode;
	int ret;

	/* Accept 0/1, "short"/"long" */
	if (sysfs_streq(buf, "short") || sysfs_streq(buf, "0"))
		mode = 0;
	else if (sysfs_streq(buf, "long") || sysfs_streq(buf, "1"))
		mode = 1;
	else {
		ret = kstrtouint(buf, 0, &mode);
		if (ret)
			return ret;
	}

	guard(mutex)(&pic->lock);
	ret = qnap_ts228_pic_buzzer(pic, mode);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(buzzer);

static struct attribute *qnap_ts228_pic_attrs[] = {
	&dev_attr_led_status.attr,
	&dev_attr_led_usb.attr,
	&dev_attr_serial_number.attr,
	&dev_attr_mac_address.attr,
	&dev_attr_power_recovery.attr,
	&dev_attr_buzzer.attr,
	NULL,
};
ATTRIBUTE_GROUPS(qnap_ts228_pic);

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

static int qnap_ts228_pic_get_fan_rpm(struct qnap_ts228_pic *pic)
{
	if (!pic->fan_valid)
		return -EAGAIN;

	return pic->fan_rpm;
}

static int qnap_ts228_pic_get_temp(struct qnap_ts228_pic *pic)
{
	if (!pic->temp_valid)
		return -EAGAIN;

	return pic->temp_millic;
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
	int ret;

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
	case hwmon_fan:
		ret = qnap_ts228_pic_get_fan_rpm(pic);
		if (ret < 0)
			return ret;
		*val = ret;
		return 0;
	case hwmon_temp:
		ret = qnap_ts228_pic_get_temp(pic);
		if (ret < 0)
			return ret;
		*val = ret;
		return 0;
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
	case hwmon_temp:
	case hwmon_fan:
		return 0444;
	case hwmon_pwm:
		return 0644;
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
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
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
		u8 raw;

		/* Stock pic_handle_cmd(0xa1) runs an 0xf6 sync read, not UART 0xa1. */
		if (!qnap_ts228_pic_read_cmd(pic, PIC_CMD_READ_FAN, &raw)) {
			pic->fan_rpm = raw * 60;
			pic->fan_valid = true;
		}
		if (!qnap_ts228_pic_read_cmd(pic, PIC_CMD_READ_TEMP, &raw)) {
			pic->temp_millic = raw * 1000;
			pic->temp_valid = true;
		}

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

	pic->monitor_task = kthread_run(qnap_ts228_pic_monitor, pic,
					"qnap-ts228-pic");
	if (IS_ERR(pic->monitor_task))
		return dev_err_probe(dev, PTR_ERR(pic->monitor_task),
				     "Failed to start PIC monitor thread\n");

	ret = devm_add_action_or_reset(dev, qnap_ts228_pic_stop_monitor, pic);
	if (ret)
		return ret;

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
	.driver = {
		.name = "qnap-ts228-pic",
		.of_match_table = qnap_ts228_pic_of_match,
		.dev_groups = qnap_ts228_pic_groups,
		.shutdown = qnap_ts228_pic_shutdown,
	},
};
module_serdev_device_driver(qnap_ts228_pic_driver);

MODULE_AUTHOR("Stephan <linux-qnap-ts228>");
MODULE_DESCRIPTION("QNAP TS-228 PIC hwmon and LED driver");
MODULE_LICENSE("GPL");
