// SPDX-License-Identifier: GPL-2.0-only
/*
 * UART probe for QNAP TS-228 PIC on /dev/ttyS1 @ 19200 8N1.
 *
 * Protocol (from QTS libuLinux_hal.so pic_sys_start_monitor / pic_send):
 *  - Init: write 0xf2
 *  - Read path: 0xf6, wait ack 0xaa, write cmd (0xf7 fan / 0xf8 temp), wait 0xaa, read byte
 *  - Fan set: single byte 0x30..0x35 (no 0xf6 framing)
 *
 * Usage:
 *   uart_probe init
 *   uart_probe read-fan
 *   uart_probe read-temp
 *   uart_probe fan <0-5>
 *   uart_probe raw <hexbytes...>
 */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>

static speed_t baud_to_flag(int baud)
{
	switch (baud) {
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
	default: return B115200;
	}
}

static int setup_tty(int fd, int baud)
{
	struct termios tio;

	if (tcgetattr(fd, &tio) < 0)
		return -1;

	cfmakeraw(&tio);
	cfsetispeed(&tio, baud_to_flag(baud));
	cfsetospeed(&tio, baud_to_flag(baud));
	tio.c_cflag |= (CLOCAL | CREAD);
	tio.c_cflag &= ~CRTSCTS;
	tio.c_cflag &= ~PARENB;
	tio.c_cflag &= ~CSTOPB;
	tio.c_cflag &= ~CSIZE;
	tio.c_cflag |= CS8;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &tio) < 0)
		return -1;

	tcflush(fd, TCIOFLUSH);
	return 0;
}

static int read_wait(int fd, unsigned int timeout_ms, unsigned char *buf, size_t len)
{
	size_t total = 0;
	struct timeval start, now, tv;
	fd_set rfds;

	gettimeofday(&start, NULL);

	while (total < len) {
		gettimeofday(&now, NULL);
		long elapsed = (now.tv_sec - start.tv_sec) * 1000L +
			       (now.tv_usec - start.tv_usec) / 1000L;
		long left = (long)timeout_ms - elapsed;

		if (left <= 0)
			break;

		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		tv.tv_sec = left / 1000;
		tv.tv_usec = (left % 1000) * 1000;

		int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
		if (ret <= 0)
			break;

		ssize_t n = read(fd, buf + total, len - total);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (n == 0)
			break;
		total += n;
	}

	return total;
}

static void hexdump(const unsigned char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		printf("%02x ", buf[i]);
	printf("\n");
}

static int write_byte(int fd, unsigned char b)
{
	if (write(fd, &b, 1) != 1)
		return -1;
	return 0;
}

static int wait_ack(int fd, unsigned int timeout_ms)
{
	unsigned long elapsed = 0;

	while (elapsed < timeout_ms) {
		unsigned char b;
		int n;
		fd_set rfds;
		struct timeval tv;

		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		tv.tv_sec = 0;
		tv.tv_usec = 50000;
		if (select(fd + 1, &rfds, NULL, NULL, &tv) <= 0) {
			elapsed += 50;
			continue;
		}
		n = read(fd, &b, 1);
		if (n == 1 && (b == 0xaa || b == 0x00))
			return 0;
		elapsed += 50;
	}
	return -ETIMEDOUT;
}

static int pic_eeprom_read(int fd, unsigned int offset, unsigned int length)
{
	unsigned char tx[3];
	unsigned char rx[128];
	unsigned int i, got = 0;
	unsigned int csum, calc;

	if (length == 0 || length + 1 > sizeof(rx))
		return -EINVAL;

	tcflush(fd, TCIFLUSH);
	if (write_byte(fd, 0xf6) < 0)
		return -1;
	if (wait_ack(fd, 1000) < 0) {
		printf("no ack after 0xf6\n");
		return -1;
	}

	tx[0] = 0xa1;
	tx[1] = offset & 0xff;
	tx[2] = length & 0xff;
	if (write(fd, tx, 3) != 3)
		return -1;
	tcdrain(fd);
	usleep(10000);
	if (wait_ack(fd, 1000) < 0) {
		printf("no ack after 0xa1 off=0x%02x len=%u\n", offset, length);
		return -1;
	}

	while (got < length + 1) {
		int n = read_wait(fd, 500, rx + got, (length + 1) - got);

		if (n <= 0)
			break;
		got += n;
	}

	printf("eeprom[0x%02x,%u] RX (%u): ", offset, length, got);
	hexdump(rx, got > 64 ? 64 : (int)got);
	if (got < length + 1) {
		printf("short read\n");
		return -1;
	}

	calc = 0;
	for (i = 0; i < length; i++)
		calc = (calc + rx[i]) & 0xff;
	csum = rx[length];
	printf("payload: ");
	for (i = 0; i < length; i++) {
		unsigned char c = rx[i];

		putchar((c >= 32 && c < 127) ? c : '.');
	}
	printf("\ncsum=0x%02x calc=0x%02x %s\n", csum, calc,
	       csum == calc ? "OK" : "MISMATCH");
	return 0;
}

static int pic_init(int fd)
{
	unsigned char rx[256];
	int len;

	if (write_byte(fd, 0xf2) < 0)
		return -1;
	usleep(200000);
	len = read_wait(fd, 1000, rx, sizeof(rx));
	printf("init RX (%d): ", len);
	if (len > 0)
		hexdump(rx, len);
	else
		printf("(none)\n");
	return 0;
}

static int pic_cmd_read(int fd, unsigned char cmd, unsigned char *out)
{
	unsigned char rx[64];
	int len;

	if (write_byte(fd, 0xf6) < 0)
		return -1;
	if (wait_ack(fd, 1000) < 0) {
		printf("no ack after 0xf6\n");
		return -1;
	}
	if (write_byte(fd, cmd) < 0)
		return -1;
	if (wait_ack(fd, 1000) < 0) {
		printf("no ack after cmd 0x%02x\n", cmd);
		return -1;
	}
	len = read_wait(fd, 1000, rx, sizeof(rx));
	if (len <= 0)
		return -1;
	*out = rx[0];
	printf("cmd 0x%02x RX (%d): ", cmd, len);
	hexdump(rx, len);
	return 0;
}

static int pic_set_fan(int fd, int level)
{
	unsigned char cmd;

	if (level < 0 || level > 5)
		return -EINVAL;
	cmd = 0x30 + level;
	printf("fan set level %d -> 0x%02x\n", level, cmd);
	return write_byte(fd, cmd);
}

int main(int argc, char **argv)
{
	const char *dev = "/dev/ttyS1";
	int baud = 19200;
	int fd;
	int i;

	for (i = 1; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-d") && i + 1 < argc)
			dev = argv[++i];
		else if (!strcmp(argv[i], "-b") && i + 1 < argc)
			baud = atoi(argv[++i]);
		else {
			fprintf(stderr, "Usage: %s [-d dev] [-b baud] init|read-fan|read-temp|fan <0-5>|eeprom <off> <len>|raw <hex...>\n",
				argv[0]);
			return 2;
		}
	}

	if (i >= argc) {
		fprintf(stderr, "missing command\n");
		return 2;
	}

	fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		perror(dev);
		return 1;
	}

	if (setup_tty(fd, baud) < 0) {
		perror("tcsetattr");
		close(fd);
		return 1;
	}

	if (!strcmp(argv[i], "init")) {
		pic_init(fd);
	} else if (!strcmp(argv[i], "read-fan")) {
		pic_init(fd);
		unsigned char b;

		if (pic_cmd_read(fd, 0xf7, &b) == 0)
			printf("fan_rpm ~ %u\n", (unsigned)(b * 60U));
	} else if (!strcmp(argv[i], "read-temp")) {
		pic_init(fd);
		unsigned char b;

		if (pic_cmd_read(fd, 0xf8, &b) == 0)
			printf("temp ~ %u C (raw byte %u)\n", b, b);
	} else if (!strcmp(argv[i], "fan")) {
		if (i + 1 >= argc) {
			fprintf(stderr, "fan level required\n");
			close(fd);
			return 2;
		}
		pic_init(fd);
		if (pic_set_fan(fd, atoi(argv[i + 1])) < 0) {
			perror("fan set");
			close(fd);
			return 1;
		}
		sleep(3);
	} else if (!strcmp(argv[i], "eeprom")) {
		unsigned int off, len;

		if (i + 2 >= argc) {
			fprintf(stderr, "eeprom <offset> <length> required\n");
			close(fd);
			return 2;
		}
		off = strtoul(argv[i + 1], NULL, 0);
		len = strtoul(argv[i + 2], NULL, 0);
		pic_init(fd);
		if (pic_eeprom_read(fd, off, len) < 0) {
			close(fd);
			return 1;
		}
	} else if (!strcmp(argv[i], "raw")) {
		unsigned char tx[64];
		int tx_len = 0;

		for (int j = i + 1; j < argc; j++) {
			unsigned long v = strtoul(argv[j], NULL, 16);
			if (v > 0xff || tx_len >= (int)sizeof(tx)) {
				fprintf(stderr, "invalid byte: %s\n", argv[j]);
				close(fd);
				return 1;
			}
			tx[tx_len++] = (unsigned char)v;
		}
		printf("TX (%d): ", tx_len);
		hexdump(tx, tx_len);
		write(fd, tx, tx_len);
		tcdrain(fd);
		usleep(200000);
		unsigned char rx[256];
		int rx_len = read_wait(fd, 500, rx, sizeof(rx));
		printf("RX (%d): ", rx_len);
		if (rx_len > 0)
			hexdump(rx, rx_len);
		else
			printf("(none)\n");
	} else {
		fprintf(stderr, "unknown command: %s\n", argv[i]);
		close(fd);
		return 2;
	}

	close(fd);
	return 0;
}
