/*
 * int memcpy(void *vdst, const void *vsrc, size_t n)
 *
 * Written July 2025, Jason R. Thorpe.
 * Public domain.
 */

#include "syslib.h"

void *
memmove(void *vdst, const void *vsrc, size_t n)
{
	char *dst = vdst;
	const char *src = vsrc;

	if (src == dst || n == 0) {
		return vdst;
	}

	if (src < dst) {
		src += n;
		dst += n;
		while (n--) {
			*--dst = *--src;
		}
	} else {
		while (n--) {
			*dst++ = *src++;
		}
	}

	return vdst;
}
