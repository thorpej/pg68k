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
#include "sysfile.h"

#ifdef CONFIG_DEV_ATA
extern const struct devsw ata_devsw;
#endif

const struct devsw *devsw[] = {
#ifdef CONFIG_DEV_ATA
	&ata_devsw,
#endif
	NULL,
};
const int ndevsw = arraycount(devsw);

static const struct devsw *
devlookup(const char *name, size_t namelen)
{
	for (int i = 0; i < ndevsw; i++) {
		if (devsw[i] == NULL) {
			continue;
		}
		if (strlen(DEV_NAME(devsw[i])) == namelen &&
		    strncmp(DEV_NAME(devsw[i]), name, namelen) == 0) {
			return devsw[i];
		}
	}
	return NULL;
}

static void
devusage(const struct devsw *dv)
{
	printf("usage: %s(", DEV_NAME(dv));
	if (dv->dv_nargs >= 1) {
		printf("ctlr");
	}
	if (dv->dv_nargs >= 2) {
		printf(",unit");
	}
	if (dv->dv_nargs >= 3) {
		printf(",part");
	}
	printf(")[/path]\n");
}

const char *
dev_string(struct open_file *f, char *buf, size_t buflen)
{
	const struct devsw *dv = f->f_dev;
	char *obuf = buf;
	char *bufend = buf + buflen;
	int rv;

#define	ADVANCE								\
	buf += rv;							\
	buflen -= rv;							\
	if (buf >= bufend) {						\
		return obuf;						\
	}

	rv = snprintf(buf, buflen, "%s(", DEV_NAME(dv));
	ADVANCE;
	if (dv->dv_nargs >= 1) {
		if (f->f_devctlr != -1) {
			rv = snprintf(buf, buflen, "%d", f->f_devctlr);
			ADVANCE;
		}
	}
	if (dv->dv_nargs >= 2) {
		if (f->f_devunit != -1) {
			rv = snprintf(buf, buflen, ",%d", f->f_devunit);
		} else {
			rv = snprintf(buf, buflen, ",");
		}
		ADVANCE;
	}
	if (dv->dv_nargs >= 3) {
		if (f->f_devpart != -1) {
			rv = snprintf(buf, buflen, ",%d", f->f_devpart);
		} else {
			rv = snprintf(buf, buflen, ",");
		}
		ADVANCE;
	}
	snprintf(buf, buflen, ")");
	return obuf;

#undef ADVANCE
}

static void
deverr(struct open_file *f, int error)
{
	char devstr[DEV_STRING_SIZE];

	printf("%s: %s\n", dev_string(f, devstr, sizeof(devstr)),
	    strerror(error));
}

/*
 * Parse a device spec in the form:
 *
 *	dev(ctlr,unit,part)
 *
 * Where:
 *
 *	dev - driver name, e.g. "ata" or "eth".
 *
 *	ctlr - the controller instance
 *
 *	unit - the specific unit on that controller instance, e.g. drive
 *	0 or drive 1 on an ATA controller.
 *
 *	part - a partition number, which 0 being the first partition of
 *	the device.
 *
 * Unspecified parameters are returned as -1.
 *
 * Examples:
 *
 *	eth(0) - the first Ethernet controller
 *
 *	eth() - also the first Ethernet controller
 *
 *	ata(0,1,0) - The first partition on drive 1 (the second drive) of
 *	the first ATA disk interface.
 *
 * Not all devices support all arguments; if a device specfies that it
 * supports "part", it must also support "unit" and "ctlr".
 *
 * Watever is left after the device identifier is passed along as the
 * file name to the file system.
 */
static int
devparse(const char *str, struct open_file *f, const char **fnamep)
{
	const char *cp;
	int *devargs[] = {
		&f->f_devctlr,
		&f->f_devunit,
		&f->f_devpart,
	};
	int val, argno, maxargs;

	/* get device name */
	for (cp = str;
	     *cp != '\0' && *cp != '/' && *cp != '(';
	     cp++) {
		continue;
	}

	if (*cp == '/') {
		/* Just got a file name. */
		*fnamep = cp;
		return 0;
	}

	if (*cp != '(') {
		/* Not sure what we got, give back the whole string. */
		*fnamep = str;
		return 0;
	}

	/* Get the driver entry. */
	f->f_dev = devlookup(str, cp - str);
	if (f->f_dev == NULL) {
		/* Don't have specified driver, oops. */
		printf("Unknown device: ");
		putstrn(str, cp - str);
		putchar('\n');
		return ENXIO;
	}
	maxargs = f->f_dev->dv_nargs;
	if (maxargs < 0) {
		maxargs = 0;
	} else if (maxargs > 3) {
		maxargs = 3;
	}

	for (cp++, val = -1, argno = 0; argno < maxargs; cp++) {
		if (*cp >= '0' && *cp <= '9') {
			if (val == -1) {
				val = 0;
			}
			val = (val * 10) + (*cp - '0');
		} else if (*cp == ',') {
			*devargs[argno] = val;
			val = -1;
			argno++;
		} else if (*cp == ')') {
			*devargs[argno] = val;
			break;
		} else {
			goto bad;
		}
	}
	if (*cp != ')') {
 bad:
		devusage(f->f_dev);
		return EINVAL;
	}
	*fnamep = ++cp;
	return 0;
}

int
dev_open(struct open_file *f, const char *path, const char **fnamep)
{
	int error;
	bool need_close = false;

	f->f_devctlr = f->f_devunit = f->f_devpart = -1;
	f->f_dev = NULL;

	error = devparse(path, f, fnamep);
	if (error) {
		return error;
	}

	/*
	 * Default the controller and unit numbers to 0.  Partition
	 * number gets to say unspecified, hinting that the partition
	 * code should search for a good candidate.
	 */
	if (f->f_devctlr == -1) {
		f->f_devctlr = 0;
	}
	if (f->f_devunit == -1) {
		f->f_devunit = 0;
	}

	error = DEV_OPEN(f->f_dev)(f);
	if (error == 0) {
		char devstr[DEV_STRING_SIZE];
		need_close = true;
		error = partition_list_scan(f, &f->f_partitions);
		if (error == 0) {
			printf("Found %s partition scheme\n",
			    partition_scheme_name(f->f_partitions.pl_scheme));
			error = partition_list_choose(&f->f_partitions,
			    f->f_devpart);
			if (error) {
				partition_list_discard(&f->f_partitions);
			}
			f->f_devpart = f->f_partitions.pl_chosen->p_partnum;
			printf("Partitions on %s:\n", dev_string(f,
			    devstr, sizeof(devstr)));
			partition_list_show(&f->f_partitions);
		}
	}
	if (error) {
		deverr(f, error);
		if (need_close) {
			DEV_CLOSE(f->f_dev)(f);
		}
		f->f_dev = NULL;
	}
	return error;
}

int
dev_close(struct open_file *f)
{
	partition_list_discard(&f->f_partitions);
	int rv = DEV_CLOSE(f->f_dev)(f);
	f->f_dev = NULL;
	return rv;
}

int
dev_read(struct open_file *f, uint64_t blkno, void *buf, size_t sz)
{
	size_t actual = 0;
	int error;

	error = DEV_STRATEGY(f->f_dev)(f, F_READ, blkno, sz, buf, &actual);
	if (error == 0 && actual != sz) {
		error = EIO;
	}
	return error;
}

int
dev_write(struct open_file *f, uint64_t blkno, const void *buf, size_t sz)
{
	size_t actual = 0;
	int error;

	error = DEV_STRATEGY(f->f_dev)(f, F_WRITE, blkno, sz,
	    UNCONST(buf)/*XXX*/, &actual);
	if (error == 0 && actual != sz) {
		error = EIO;
	}
	return error;
}

size_t
getsecsize(struct open_file *f)
{
	return DEV_BSIZE;	/* XXX for now */
}
