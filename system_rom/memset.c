/*
 * int memset(void *vdst, int ch, size_t n)
 *
 * Written July 2025, Jason R. Thorpe.
 * Public domain.
 */

#include "syslib.h"

void *
memset(void *vdst, int ch, size_t n)
{
	char *dst = vdst;

	while (n--) {
		*dst++ = (char)ch;
	}

	return vdst;
}
