/*	From NetBSD: open.c,v 1.26 2007/11/24 13:20:56 isaki Exp	*/
/*	From NetBSD: close.c,v 1.14 2007/12/02 04:59:25 tsutsui Exp	*/
/*	From NetBSD: closeall.c,v 1.5 2007/11/24 13:20:54 isaki Exp	*/
/*	From NetBSD: read.c,v 1.15 2007/12/02 04:59:26 tsutsui Exp	*/
/*	From NetBSD: write.c,v 1.15 2007/12/02 04:59:26 tsutsui Exp	*/
/*	from NetBSD: lseek.c,v 1.11 2007/12/02 04:59:26 tsutsui Exp	*/
/*	From NetBSD: stat.c,v 1.7 2007/11/24 13:20:57 isaki Exp		*/
/*	From NetBSD: fstat.c,v 1.7 2007/12/02 04:59:25 tsutsui Exp	*/
/*	From NetBSD: ioctl.c,v 1.11 2007/12/02 04:59:25 tsutsui Exp	*/

/*-
 * Copyright (c) 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * The Mach Operating System project at Carnegie-Mellon University.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)open.c	8.1 (Berkeley) 6/11/93
 *	@(#)close.c	8.1 (Berkeley) 6/11/93
 *	@(#)read.c	8.1 (Berkeley) 6/11/93
 *	@(#)write.c	8.1 (Berkeley) 6/11/93
 *	@(#)lseek.c	8.1 (Berkeley) 6/11/93
 *	@(#)stat.c	8.1 (Berkeley) 6/11/93
 *	@(#)ioctl.c	8.1 (Berkeley) 6/11/93
 *
 *
 * Copyright (c) 1989, 1990, 1991 Carnegie Mellon University
 * All Rights Reserved.
 *
 * Author: Alessandro Forin
 *
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

#include "syslib.h"
#include "sysfile.h"

static struct open_file files[SOPEN_MAX];

/*
 *	File primitives proper
 */

int
open(const char *fname, int mode)
{
	struct open_file *f;
	int fd, error;
	int i, besterror;
	char *file;

	/* find a free file descriptor */
	for (fd = 0, f = files; fd < SOPEN_MAX; fd++, f++)
		if (f->f_flags == 0)
			goto fnd;
	errno = EMFILE;
	return -1;
fnd:
	/*
	 * Try to open the device.
	 * Convert open mode (0,1,2) to F_READ, F_WRITE.
	 */
	f->f_flags = mode + 1;
	f->f_dev = NULL;
	f->f_ops = NULL;
	f->f_offset = 0;

	file = NULL;
	error = devopen(f, fname, &file);
	if (error
	    || (((f->f_flags & F_NODEV) == 0) &&
		f->f_dev == NULL)
	    )
		goto err;

	/* see if we opened a raw device; otherwise, 'file' is the file name. */
	if (file == NULL || *file == '\0') {
		f->f_flags |= F_RAW;
		return fd;
	}

	/* pass file name to the different filesystem open routines */
	besterror = ENOENT;
	for (i = 0; i < nfsys; i++) {
		if (file_systems[i] == NULL) {
			continue;
		}
		error = FS_OPEN(file_systems[i])(file, f);
		if (error == 0) {
			f->f_ops = file_systems[i];
			return fd;
		}
		if (error != EINVAL)
			besterror = error;
	}
	error = besterror;

	if ((f->f_flags & F_NODEV) == 0) {
		if (DEV_CLOSE(f->f_dev) != NULL)
			(void)DEV_CLOSE(f->f_dev)(f);
	}
err:
	f->f_flags = 0;
	errno = error;
	return -1;
}

int
getfile(int fd, struct open_file **fp)
{
	struct open_file *f = &files[fd];

	if ((unsigned int)fd >= SOPEN_MAX || f->f_flags == 0) {
		errno = EBADF;
		return -1;
	}

	*fp = f;
	return 0;
}

int
close(int fd)
{
	struct open_file *f = &files[fd];
	int err1 = 0, err2 = 0;

	if ((unsigned int)fd >= SOPEN_MAX || f->f_flags == 0) {
		errno = EBADF;
		return -1;
	}

	if (!(f->f_flags & F_RAW))
		if (f->f_ops != NULL)
			err1 = FS_CLOSE(f->f_ops)(f);
	if (!(f->f_flags & F_NODEV))
		if (f->f_dev != NULL)
			err2 = DEV_CLOSE(f->f_dev)(f);
	f->f_flags = 0;
	if (err1) {
		errno = err1;
		return -1;
	}
	if (err2) {
		errno = err2;
		return -1;
	}
	return 0;
}

void
closeall(void)
{
	int i;

	for (i = 0; i < SOPEN_MAX; i++)
		if (files[i].f_flags != 0)
			(void)close(i);
}

ssize_t
read(int fd, void *dest, size_t bcount)
{
	struct open_file *f = &files[fd];
	size_t resid;

	if ((unsigned int)fd >= SOPEN_MAX || !(f->f_flags & F_READ)) {
		errno = EBADF;
		return -1;
	}
	if (f->f_flags & F_RAW) {
		errno = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
			btodb(f->f_offset), bcount, dest, &resid);
		if (errno)
			return -1;
		f->f_offset += resid;
		return resid;
	}
	resid = bcount;
	if ((errno = FS_READ(f->f_ops)(f, dest, bcount, &resid)))
		return -1;
	return (ssize_t)(bcount - resid);
}

ssize_t
write(int fd, const void *destp, size_t bcount)
{
	struct open_file *f = &files[fd];
	size_t resid;
	void *dest = UNCONST(destp);

	if ((unsigned int)fd >= SOPEN_MAX || !(f->f_flags & F_WRITE)) {
		errno = EBADF;
		return -1;
	}
	if (f->f_flags & F_RAW) {
		errno = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_WRITE,
			btodb(f->f_offset), bcount, dest, &resid);
		if (errno)
			return -1;
		f->f_offset += resid;
		return resid;
	}
	resid = bcount;
	if ((errno = FS_WRITE(f->f_ops)(f, dest, bcount, &resid)))
		return -1;
	return 0;
}

off_t
lseek(int fd, off_t offset, int where)
{
	struct open_file *f = &files[fd];

	if ((unsigned int)fd >= SOPEN_MAX || f->f_flags == 0) {
		errno = EBADF;
		return -1;
	}

	if (f->f_flags & F_RAW) {
		/*
		 * On RAW devices, update internal offset.
		 */
		switch (where) {
		case SEEK_SET:
			f->f_offset = offset;
			break;
		case SEEK_CUR:
			f->f_offset += offset;
			break;
		case SEEK_END:
		default:
			errno = EINVAL;
			return -1;
		}
		return f->f_offset;
	}

	return FS_SEEK(f->f_ops)(f, offset, where);
}

int
stat(const char *str, struct stat *sb)
{
	int fd, rv;

	fd = open(str, 0);
	if (fd < 0)
		return -1;
	rv = fstat(fd, sb);
	(void)close(fd);
	return rv;
}

int
fstat(int fd, struct stat *sb)
{
	struct open_file *f = &files[fd];

	if ((unsigned int)fd >= SOPEN_MAX || f->f_flags == 0) {
		errno = EBADF;
		return -1;
	}

	/* operation not defined on raw devices */
	if (f->f_flags & F_RAW) {
		errno = EOPNOTSUPP;
		return -1;
	}

	errno = FS_STAT(f->f_ops)(f, sb);	/* XXX no point setting errno */
	return 0;
}

int
ioctl(int fd, u_long cmd, void *arg)
{
	struct open_file *f = &files[fd];

	if ((unsigned int)fd >= SOPEN_MAX || f->f_flags == 0) {
		errno = EBADF;
		return -1;
	}

	if (f->f_flags & F_RAW) {
		errno = DEV_IOCTL(f->f_dev)(f, cmd, arg);
		if (errno)
			return -1;
		return 0;
	}

	errno = EIO;
	return -1;
}
