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

int	memcmp(const void *, const void *, size_t);
void *	memcpy(void *, const void *, size_t);
void *	memset(void *, int, size_t);

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
