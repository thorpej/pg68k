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

#ifndef systypes_h_included
#define	systypes_h_included

#ifdef CONFIG_MACH_HOST_SIM

#include <sys/types.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>

/* XXX */
typedef long long int		longlong_t;
typedef unsigned long long int	u_longlong_t;

#else /* ! CONFIG_MACH_HOST_SIM */

#include "cdefs.h"

#define	bool			_Bool
#define	true			1
#define	false			0

#define	NULL			((void *)0)
#define	NBBY			8

typedef __INT8_TYPE__		int8_t;
typedef __UINT8_TYPE__		uint8_t;

typedef __INT16_TYPE__		int16_t;
typedef __UINT16_TYPE__		uint16_t;

typedef __INT32_TYPE__		int32_t;
typedef __UINT32_TYPE__		uint32_t;

typedef __INT64_TYPE__		int64_t;
typedef __UINT64_TYPE__		uint64_t;

typedef __SIZE_TYPE__		size_t;
typedef __PTRDIFF_TYPE__	ssize_t;
typedef __PTRDIFF_TYPE__	ptrdiff_t;
typedef __INTPTR_TYPE__		intptr_t;
typedef __UINTPTR_TYPE__	uintptr_t;

typedef __INTMAX_TYPE__		intmax_t;
typedef __UINTMAX_TYPE__	uintmax_t;

#ifndef __VA_LIST_DECLARED
typedef __builtin_va_list	va_list;
#define	__VA_LIST_DECLARED
#endif

typedef unsigned char		u_char;
typedef unsigned short int	u_short;
typedef unsigned int		u_int;
typedef unsigned long int	u_long;
typedef long long int		longlong_t;
typedef unsigned long long int	u_longlong_t;

typedef volatile void *		vptr_t;
typedef volatile uint8_t *	vptr8_t;
typedef volatile uint16_t *	vptr16_t;
typedef volatile uint32_t *	vptr32_t;

typedef uint32_t		in_addr_t;
typedef uint16_t		in_port_t;

#define	_JBLEN			21
typedef long			jmp_buf[_JBLEN];

#endif /* CONFIG_MACH_HOST_SIM */

#endif /* systypes_h_included */
