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

#ifndef CONFIG_MACH_HOST_SIM
#define	offsetof(s, m)	__builtin_offsetof(s, m)
#endif

/*
 * The following macro is used to remove const cast-away warnings
 * from gcc -Wcast-qual; it should be used with caution because it
 * can hide valid errors; in particular most valid uses are in
 * situations where the API requires it, not to cast away string
 * constants. We don't use *intptr_t on purpose here and we are
 * explicit about unsigned long so that we don't have additional
 * dependencies.
 */
#define UNCONST(a)	((void *)(uintptr_t)(const void *)(a))

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
#define	EBUSY		7	/* Device busy */
#define	EEXIST		8	/* File exists */
#define	ENODEV		9	/* Operation not supported by device */
#define	ENOTDIR		10	/* /* Not a directory */
#define	EINVAL		11	/* Invalid argument */
#define	EMFILE		12	/* Too many open files */
#define	EFBIG		13	/* File too large */
#define	ENOSPC		14	/* No space left on device */
#define	EROFS		15	/* Read-only file system */
#define	EMLINK		16	/* Too many links */
#define	EOPNOTSUPP	17	/* Operation not supported */
#define	ELOOP		18	/* Too many levels of symbolic links */
#define	ENAMETOOLONG	19	/* File name too long */
#define	EFTYPE		20	/* Inappropriate file type or format */

extern int errno;
#endif /* ! CONFIG_MACH_HOST_SIM */

char *	strerror(int);

#define	ffs(x)		__builtin_ffs(x)

static inline const char *
plural(long v)
{
	return v == 1 ? "" : "s";
}

int	memcmp(const void *, const void *, size_t);
void *	memcpy(void *, const void *, size_t);
void *	memmove(void *, const void *, size_t);
void *	memset(void *, int, size_t);

char *	strchr(const char *, int);
char *	strrchr(const char *, int);
int	strcmp(const char *, const char *);
int	strncmp(const char *, const char *, size_t);
size_t	strlen(const char *);

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
int	putstrn(const char *, size_t);
int	putchar(int);

int	ucs2_to_utf8(char *, size_t, uint16_t);

#endif /* syslib_h_included */
