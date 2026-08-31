# Userspace helpers for QNAP TS-228 PIC (cross-build for ARM NAS)
#
#   make -C tools/qnap-ts228 uart_probe
#   make -C tools/qnap-ts228 deploy HOST=root@192.168.178.96

CROSS_COMPILE ?= arm-none-linux-gnueabihf-
CC            := $(CROSS_COMPILE)gcc
CFLAGS        := -O2 -Wall -Wextra -static

ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
UART_PROBE := $(ROOT)uart_probe
HOST ?= root@192.168.178.96

.PHONY: all uart_probe deploy clean

all: uart_probe

uart_probe: uart_probe.c
	$(CC) $(CFLAGS) -o $@ $<

deploy: uart_probe
	scp -o StrictHostKeyChecking=no $(UART_PROBE) $(HOST):/tmp/uart_probe
	ssh -o StrictHostKeyChecking=no $(HOST) chmod +x /tmp/uart_probe
	@echo "Deployed: /tmp/uart_probe on $(HOST)"

clean:
	rm -f uart_probe
