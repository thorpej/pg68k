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

static void
deverr(const struct devsw *dv, int ctlr, int unit, int part, int error)
{
	printf("%s(", DEV_NAME(dv));
	if (dv->dv_nargs >= 1) {
		printf("%d", ctlr);
	}
	if (dv->dv_nargs >= 2) {
		printf(",%d", unit);
	}
	if (dv->dv_nargs >= 3) {
		printf(",%d", part);
	}
	printf("): %s\n", strerror(error));
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
devparse(const char *str, const struct devsw **dvp,
    int *ctlrp, int *unitp, int *partp, const char **fnamep)
{
	const char *cp;
	int *devargs[] = {
		ctlrp,
		unitp,
		partp,
	};
	const struct devsw *dv;
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
	dv = devlookup(str, cp - str);
	if (dv == NULL) {
		/* Don't have specified driver, oops. */
		printf("Unknown device: ");
		putstrn(str, cp - str);
		putchar('\n');
		return ENXIO;
	}
	*dvp = dv;
	maxargs = dv->dv_nargs;
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
		devusage(dv);
		return EINVAL;
	}
	*fnamep = ++cp;
	return 0;
}

int
dev_open(struct open_file *f, const char *path, const char **fnamep)
{
	int ctlr = -1, unit = -1, part = -1;
	int error;
	const struct devsw *dv = NULL;
	bool need_close = false;

	error = devparse(path, &dv, &ctlr, &unit, &part, fnamep);
	if (error) {
		return error;
	}

	/*
	 * Default the controller and unit numbers to 0.  Partition
	 * number gets to say unspecified, hinting that the partition
	 * code should search for a good candidate.
	 */
	if (ctlr == -1) {
		ctlr = 0;
	}
	if (unit == -1) {
		unit = 0;
	}

	f->f_dev = dv;
	error = DEV_OPEN(dv)(f, ctlr, unit, part);
	if (error == 0) {
		need_close = true;
		error = partition_list_scan(f, &f->f_partitions);
		if (error == 0) {
			printf("Found %s partition scheme",
			    partition_scheme_name(f->f_partitions.pl_scheme));
			error = partition_list_choose(&f->f_partitions, part);
			if (error) {
				partition_list_discard(&f->f_partitions);
			}
		}
	}
	if (error) {
		deverr(dv, ctlr, unit, part, error);
		if (need_close) {
			DEV_CLOSE(dv)(f);
		}
	}
	return error;
}

int
dev_close(struct open_file *f)
{
	partition_list_discard(&f->f_partitions);
	return DEV_CLOSE(f->f_dev)(f);
}

int
dev_read(struct open_file *f, uint64_t blkno, void *buf, size_t sz)
{
	size_t resid = sz;
	int error;

	error = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ, blkno,
	    sz, buf, &resid);
	if (error == 0 && resid != 0) {
		error = EIO;
	}
	return error;
}

int
dev_write(struct open_file *f, uint64_t blkno, const void *buf, size_t sz)
{
	size_t resid = sz;
	int error;

	error = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_WRITE, blkno,
	    sz, UNCONST(buf)/*XXX*/, &resid);
	if (error == 0 && resid != 0) {
		error = EIO;
	}
	return error;
}

size_t
getsecsize(struct open_file *f)
{
	return DEV_BSIZE;	/* XXX for now */
}
