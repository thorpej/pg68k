/*	From NetBSD: subr_prf.c,v 1.31 2024/08/31 07:12:57 mlelstv Exp	*/

/*-
 * Copyright (c) 1993
 *	The Regents of the University of California.  All rights reserved.
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
 *	@(#)printf.c	8.1 (Berkeley) 6/11/93
 */

/*
 * Scaled down version of printf(3).
 */

#include "config.h"
#include "syslib.h"
#include "stdarg.h"
#include "console.h"

#define INTMAX_T	intmax_t
#define UINTMAX_T	uintmax_t
#define PTRDIFF_T	ptrdiff_t

#define	LIBSA_PRINTF_WIDTH_SUPPORT
#define	LIBSA_PRINTF_LONGLONG_SUPPORT

#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
static void kprintn(void (*)(int), UINTMAX_T, int, int, int);
#else
static void kprintn(void (*)(int), UINTMAX_T, int);
#endif
static void sputchar(int);
static void kdoprnt(void (*)(int), const char *, va_list);

static char *sbuf, *ebuf;

const char hexdigits[16] = "0123456789abcdef";

#define LONG		0x01
#ifdef LIBSA_PRINTF_LONGLONG_SUPPORT
#define LLONG		0x02
#endif
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
#define ALT		0x04
#define SPACE		0x08
#define LADJUST		0x10
#define SIGN		0x20
#define ZEROPAD		0x40
#define NEGATIVE	0x80
#define KPRINTN(base)	kprintn(put, ul, base, lflag, width)
#define RADJUSTZEROPAD()					\
do {								\
	if ((lflag & (ZEROPAD|LADJUST)) == ZEROPAD) {		\
		while (width-- > 0)				\
			put('0');				\
	}							\
} while (/*CONSTCOND*/0)
#define LADJUSTPAD()						\
do {								\
	if (lflag & LADJUST) {					\
		while (width-- > 0)				\
			put(' ');				\
	}							\
} while (/*CONSTCOND*/0)
#define RADJUSTPAD()						\
do {								\
	if ((lflag & (ZEROPAD|LADJUST)) == 0) {			\
		while (width-- > 0)				\
			put(' ');				\
	}							\
} while (/*CONSTCOND*/0)
#else	/* LIBSA_PRINTF_WIDTH_SUPPORT */
#define KPRINTN(base)	kprintn(put, ul, base)
#define RADJUSTZEROPAD()	/**/
#define LADJUSTPAD()		/**/
#define RADJUSTPAD()		/**/
#endif	/* LIBSA_PRINTF_WIDTH_SUPPORT */

#ifdef LIBSA_PRINTF_LONGLONG_SUPPORT
#define KPRINT(base)						\
do {								\
	ul = (lflag & LLONG)					\
	    ? va_arg(ap, u_longlong_t)				\
	    : (lflag & LONG)					\
		? va_arg(ap, u_long)				\
		: va_arg(ap, u_int);				\
	KPRINTN(base);						\
} while (/*CONSTCOND*/0)
#else	/* LIBSA_PRINTF_LONGLONG_SUPPORT */
#define KPRINT(base)						\
do {								\
	ul = (lflag & LONG)					\
	    ? va_arg(ap, u_long) : va_arg(ap, u_int);		\
	KPRINTN(base);						\
} while (/*CONSTCOND*/0)
#endif	/* LIBSA_PRINTF_LONGLONG_SUPPORT */

static void
null_sputchar(int c)
{
	sbuf++;
}

static void
sputchar(int c)
{

	if (sbuf < ebuf)
		*sbuf++ = c;
}

static void
cputchar(int c)
{
	cons_putc(c);
	sbuf++;
}

int
vprintf(const char *fmt, va_list ap)
{
	sbuf = NULL;
	kdoprnt(cputchar, fmt, ap);
	return (int)(sbuf - (char *)NULL); /* XXX gross, but works */
}

int
printf(const char *fmt, ...)
{
	va_list ap;
	int rv;

	va_start(ap, fmt);
	rv = vprintf(fmt, ap);
	va_end(ap);

	return rv;
}

int
puts(const char *s)
{
	int ch;
	int cnt = 0;

	while ((ch = *s++) != '\0') {
		putchar(ch);
		cnt++;
	}
	putchar('\n');
	return cnt+1;
}

/* Avoid circular dependency with uart_hostsim.c */
#ifndef CONFIG_MACH_HOST_SIM
int
putchar(int c)
{
	cons_putc(c);
	return c;
}
#endif /* ! CONFIG_MACH_HOST_SIM */

int
vsnprintf(char *buf, size_t size, const char *fmt, va_list ap)
{

	sbuf = buf;
	ebuf = buf + size - 1;
	kdoprnt(buf == NULL ? null_sputchar : sputchar, fmt, ap);
	if (buf != NULL)
		*sbuf = '\0';
	return sbuf - buf;
}

static void
kdoprnt(void (*put)(int), const char *fmt, va_list ap)
{
	char *p;
	int ch;
	UINTMAX_T ul;
	int lflag;
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
	int width;
	char *q;
#endif

	for (;;) {
		while ((ch = *fmt++) != '%') {
			if (ch == '\0')
				return;
			put(ch);
		}
		lflag = 0;
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
		width = 0;
#endif
reswitch:
		switch (ch = *fmt++) {
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
		case '#':
			lflag |= ALT;
			goto reswitch;
		case ' ':
			lflag |= SPACE;
			goto reswitch;
		case '-':
			lflag |= LADJUST;
			goto reswitch;
		case '+':
			lflag |= SIGN;
			goto reswitch;
		case '0':
			if (width == 0) {
				lflag |= ZEROPAD;
				goto reswitch;
			}
			/* FALLTHROUGH */
		case '1': case '2': case '3': case '4': case '5':
		case '6': case '7': case '8': case '9':
			for (;;) {
				width *= 10;
				width += ch - '0';
				ch = *fmt;
				if ((unsigned)ch - '0' > 9)
					break;
				++fmt;
			}
			goto reswitch;
#endif
		case 'l':
#ifdef LIBSA_PRINTF_LONGLONG_SUPPORT
			if (*fmt == 'l') {
				++fmt;
				lflag |= LLONG;
			} else
#endif
				lflag |= LONG;
			goto reswitch;
		case 'j':
#ifdef LIBSA_PRINTF_LONGLONG_SUPPORT
			if (sizeof(intmax_t) == sizeof(long long))
				lflag |= LLONG;
			else
#endif
			if (sizeof(intmax_t) == sizeof(long))
				lflag |= LONG;
			goto reswitch;
		case 't':
			if (sizeof(PTRDIFF_T) == sizeof(long))
				lflag |= LONG;
			goto reswitch;
		case 'z':
			if (sizeof(ssize_t) == sizeof(long))
				lflag |= LONG;
			goto reswitch;
		case 'c':
			ch = va_arg(ap, int);
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
			--width;
#endif
			RADJUSTPAD();
			put(ch & 0xFF);
			LADJUSTPAD();
			break;
		case 's':
			p = va_arg(ap, char *);
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
			for (q = p; *q != '\0'; ++q)
				continue;
			width -= q - p;
#endif
			RADJUSTPAD();
			while ((ch = (unsigned char)*p++))
				put(ch);
			LADJUSTPAD();
			break;
		case 'd':
			ul =
#ifdef LIBSA_PRINTF_LONGLONG_SUPPORT
			(lflag & LLONG) ? va_arg(ap, longlong_t) :
#endif
			(lflag & LONG) ? va_arg(ap, long) : va_arg(ap, int);
			if ((INTMAX_T)ul < 0) {
				ul = -(INTMAX_T)ul;
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
				lflag |= NEGATIVE;
#else
				put('-');
#endif
			}
			KPRINTN(10);
			break;
		case 'o':
			KPRINT(8);
			break;
		case 'u':
			KPRINT(10);
			break;
		case 'p':
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
			lflag |= (LONG|ALT);
#else
			put('0');
			put('x');
#endif
			/* FALLTHROUGH */
		case 'x':
			KPRINT(16);
			break;
		default:
			if (ch == '\0')
				return;
			put(ch);
			break;
		}
	}
}

static void
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
kprintn(void (*put)(int), UINTMAX_T ul, int base, int lflag, int width)
#else
kprintn(void (*put)(int), UINTMAX_T ul, int base)
#endif
{
					/* hold a INTMAX_T in base 8 */
	char *p, buf[(sizeof(INTMAX_T) * CHAR_BIT / 3) + 1 + 2 /* ALT + SIGN */];
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
	char *q;
#endif

	p = buf;
	do {
		*p++ = hexdigits[ul % base];
	} while (ul /= base);
#ifdef LIBSA_PRINTF_WIDTH_SUPPORT
	q = p;
	if (lflag & ALT && *(p - 1) != '0') {
		if (base == 8) {
			*p++ = '0';
		} else if (base == 16) {
			*p++ = 'x';
			*p++ = '0';
		}
	}
	if (lflag & NEGATIVE)
		*p++ = '-';
	else if (lflag & SIGN)
		*p++ = '+';
	else if (lflag & SPACE)
		*p++ = ' ';
	width -= p - buf;
	if (lflag & ZEROPAD) {
		while (p > q)
			put(*--p);
	}
#endif
	RADJUSTPAD();
	RADJUSTZEROPAD();
	do {
		put(*--p);
	} while (p > buf);
	LADJUSTPAD();
}
