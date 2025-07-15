/*
 * int memcpy(void *vdst, const void *vsrc, size_t n)
 *
 * Written July 2025, Jason R. Thorpe.
 * Public domain.
 */

#include "syslib.h"

void *
memcpy(void *vdst, const void *vsrc, size_t n)
{
	char *dst = vdst;
	const char *src = vsrc;

	while (n--) {
		*dst++ = *src++;
	}

	return vdst;
}
