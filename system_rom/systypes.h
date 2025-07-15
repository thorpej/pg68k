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

#include "cdefs.h"

#define	bool			_Bool
#define	true			1
#define	false			0

typedef __INT8_TYPE__		int8_t;
typedef __UINT8_TYPE__		uint8_t;

typedef short int		int16_t;
typedef unsigned short int	uint16_t;

typedef int			int32_t;
typedef unsigned int		uint32_t;

typedef long long int		int64_t;
typedef unsigned long long int	uint64_t;

typedef __SIZE_TYPE__		size_t;
typedef __PTRDIFF_TYPE__	ssize_t;
typedef __PTRDIFF_TYPE__	ptrdiff_t;
typedef __UINTPTR_TYPE__	uintptr_t;

typedef volatile void *		vptr_t;
typedef volatile uint8_t *	vptr8_t;
typedef volatile uint16_t *	vptr16_t;

typedef uint32_t		in_addr_t;
typedef uint16_t		in_port_t;

#endif /* systypes_h_included */
