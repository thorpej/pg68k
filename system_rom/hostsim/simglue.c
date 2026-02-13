/*
 * Copyright (c) 2025 Jason R. Thorpe.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "config.h"
#include "syslib.h"

#include "trap.h"
#include "simglue.h"

#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

jmp_buf nofault_env;
bool nofault;

#undef RAM0_SIZE
#define	RAM0_SIZE	(8*1024*1024)
#define	RAM0_MASK	(RAM0_SIZE - 1)

#define	RAM0_OFF(x)	(((x) - RAM0_START) & RAM0_MASK)

static uint8_t ram0[RAM0_SIZE];

bool
badaddr_read32(volatile uint32_t *p, uint32_t *valp)
{
	uintptr_t x = (uintptr_t)p;

	if (x >= RAM0_START && x < RAM0_START+RAM0_MAXSIZE) {
		memcpy(valp, &ram0[RAM0_OFF(x)], sizeof(*valp));
		return false;
	}

	return true;
}

bool
badaddr_write32(volatile uint32_t *p, uint32_t val)
{
	uintptr_t x = (uintptr_t)p;

	if (x >= RAM0_START && x < RAM0_START+RAM0_MAXSIZE) {
		memcpy(&ram0[RAM0_OFF(x)], &val, sizeof(val));
		return false;
	}

	return true;
}

static struct termios saved_termios;
static struct termios raw_termios;
static int stdout_fd;
static struct pollfd stdout_pfd;

static void
restore_stdout(void)
{
	tcsetattr(stdout_fd, TCSAFLUSH, &saved_termios);
}

void
sim_uart_init(void)
{
	stdout_fd = fileno(stdout);

	stdout_pfd.fd = stdout_fd;
	stdout_pfd.events = POLLIN | POLLOUT;

	tcgetattr(stdout_fd, &saved_termios);

	atexit(restore_stdout);

	raw_termios = saved_termios;
	cfmakeraw(&raw_termios);

#if 0	/* Don't want this right now. */
	raw_termios.c_lflag |= ISIG;
#endif
	tcsetattr(stdout_fd, TCSAFLUSH, &raw_termios);
}

int
sim_uart_pollc(void)
{
	if (poll(&stdout_pfd, 1, 0) > 0 &&
	    (stdout_pfd.revents & POLLIN) != 0) {
		uint8_t ch;

		(void) read(stdout_fd, &ch, 1);
		return ch;
	}
	return -1;
}

int
sim_uart_getc(void)
{
	uint8_t ch;

	(void) read(stdout_fd, &ch, 1);
	return ch;
}

void
sim_uart_putc(int c)
{
	uint8_t ch = (uint8_t)c;

	while (poll(&stdout_pfd, 1, -1) > 0) {
		if (stdout_pfd.revents & POLLOUT) {
			(void) write(stdout_fd, &ch, 1);
			usleep(86);	/* time between chars at 115.2k */
			return;
		}
	}
}

struct sim_ata_softc {
	int	sc_fd;
	int	sc_secsize;
	uint64_t sc_nblks;
};

static struct sim_ata_softc sim_ata_drives[2];

void
sim_ata_init(bool do_init)
{
	char drive_name[32];
	struct sim_ata_softc *sc;
	struct stat sb;

	for (int i = 0; do_init && i < 2; i++) {
		sc = &sim_ata_drives[i];
		snprintf(drive_name, sizeof(drive_name),
		    "disk%d.img", i);
		sc->sc_fd = open(drive_name, O_RDWR);
		if (sc->sc_fd == -1) {
 bad:
			memset(sc, 0, sizeof(*sc));
			continue;
		}
		sc->sc_secsize = 512;
		if (fstat(sc->sc_fd, &sb) == -1) {
			close(sc->sc_fd);
			goto bad;
		}
		sc->sc_nblks = sb.st_size / sc->sc_secsize;
	}

	for (int i = 0; i < 2; i++) {
		sc = &sim_ata_drives[i];
		if (sc->sc_nblks == 0) {
			continue;
		}
		printf("  drive %d: <Host Sim Disk> %llu %d-byte blocks\n",
		    i, (unsigned long long)sc->sc_nblks, sc->sc_secsize);
	}
}

int
sim_ata_strategy(void *arg, int flags, uint64_t blk, size_t len, void *buf,
    size_t *actualp)
{
	struct sim_ata_softc *sc = arg;
	size_t blkcnt;
	ssize_t actual;

	*actualp = 0;

	if (len % sc->sc_secsize) {
		return EINVAL;
	}
	blkcnt = len / sc->sc_secsize;

	if (blk >= sc->sc_nblks) {
		return EIO;
	}

	if ((blk + blkcnt) > sc->sc_nblks) {
		blkcnt -= (blk + blkcnt) - sc->sc_nblks;
		len = blkcnt * sc->sc_secsize;
	}

	if (flags == 1/*XXX FREAD*/) {
		actual = pread(sc->sc_fd, buf, len, blk * sc->sc_secsize);
	} else {
		actual = pwrite(sc->sc_fd, buf, len, blk * sc->sc_secsize);
	}
	if (actual < 0) {
		return EIO;
	}
	*actualp = actual;
	return 0;
}

int
sim_ata_open(int unit, void **argp)
{
	if (unit < 0 || unit > 1) {
		return ENXIO;
	}
	if (sim_ata_drives[unit].sc_nblks == 0) {
		return ENXIO;
	}
	*argp = &sim_ata_drives[unit];
	return 0;
}

int
sim_ata_close(void *arg)
{
	return 0;
}

int
sim_ata_ioctl(void *arg, unsigned long cmd, void *data)
{
	return EINVAL;
}

ssize_t
sim_loader_read(int fd, uintptr_t dst, size_t sz)
{
	return sz;
}

void
sim_loader_bcopy(const void *src, uintptr_t dst, size_t sz)
{
}

void
sim_loader_bzero(uintptr_t dst, size_t sz)
{
}

void
sim_clock_delay(int us)
{
	(void) usleep(us);
}

time_t
sim_clock_getsecs(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0) {
		return 0;
	}
	return ts.tv_sec;
}

static void *rom_fdt_store;

const void *
sim_rom_fdt(void)
{
	if (rom_fdt_store != NULL) {
		free(rom_fdt_store);
		rom_fdt_store = NULL;
	}
	int fd = open("device-tree.dtb", O_RDONLY);
	if (fd >= 0) {
		struct stat sb;
		if (fstat(fd, &sb) == 0) {
			rom_fdt_store = malloc((size_t)sb.st_size);
			if (rom_fdt_store != NULL) {
				read(fd, rom_fdt_store, (size_t)sb.st_size);
			}
		}
		close(fd);
	}
	return rom_fdt_store;
}

void
sim_boot_fdt(const void *buf, size_t buflen)
{
	int fd = open("booted-device-tree.dtb", O_RDWR|O_CREAT|O_TRUNC, 0644);
	if (fd >= 0) {
		write(fd, buf, buflen);
		close(fd);
	}
	if (rom_fdt_store != NULL) {
		free(rom_fdt_store);
		rom_fdt_store = NULL;
	}
	exit(0);
}
