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

#ifndef syslib_h_included
#define	syslib_h_included

#include "config.h"
#include "systypes.h"

#define	_REG_OFF(r)	((r) << CONFIG_DEV_REGSHIFT)

/*
 * Return the number of elements in a statically-allocated array,
 * __x.
 */
#define	arraycount(x)	(sizeof(x) / sizeof(x[0]))

/*
 * Round up a value to the next power-of-two.
 */
#define	roundup(x, y)	(((uintptr_t)(x) + ((y) - 1)) & ~((y) - 1))

/*
 * Convert to/from disk blocks (XXX assume 512 byte sectors).
 */
#define	DEV_BSHIFT	9
#define	DEV_BSIZE	(1U << DEV_BSHIFT)

#define	dbtob(x)	((x) << DEV_BSHIFT)
#define	btodb(x)	((x) >> DEV_BSHIFT)

#ifndef CONFIG_MACH_HOST_SIM
#define	ENOENT		1	/* No such file or directory */
#define	EIO		2	/* Input/output error */
#define	ENXIO		3	/* Device not configured */
#define	ENOEXEC		4	/* Exec format error */
#define	EBADF		5	/* Bad file descriptor */
#define	ENOMEM		6	/* Cannot allocate memory */
#define	EEXIST		7	/* File exists */
#define	ENODEV		8	/* Operation not supported by device */
#define	EINVAL		9	/* Invalid argument */
#define	EMFILE		10	/* Too many open files */
#define	ENOSPC		11	/* No space left on device */
#define	EROFS		12	/* Read-only file system */
#define	EOPNOTSUPP	13	/* Operation not supported */

extern int errno;
#endif /* ! CONFIG_MACH_HOST_SIM */

int	memcmp(const void *, const void *, size_t);
void *	memcpy(void *, const void *, size_t);
void *	memmove(void *, const void *, size_t);
void *	memset(void *, int, size_t);

void *	calloc(size_t, size_t);
void *	malloc(size_t);
void *	realloc(void *, size_t);
void	free(void *);

int	setjmp(jmp_buf);
void	longjmp(jmp_buf, int);

int	printf(const char *, ...) __printflike(1,2);
int	snprintf(char *, size_t, const char *, ...) __printflike(3,4);
int	vprintf(const char *, va_list) __printflike(1, 0);
int	vsnprintf(char *, size_t, const char *, va_list) __printflike(3,0);

void	panic(const char *, ...);

int	puts(const char *);
int	putchar(int);

#endif /* syslib_h_included */
