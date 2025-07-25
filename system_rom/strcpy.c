/*
 * char *strcpy(char *dst, const char *src)
 *
 * Written July 2025, Jason R. Thorpe.
 * Public domain.
 */

#include "syslib.h"

char *
strcpy(char *dst, const char *src)
{
	while ((*dst++ = *src++) != '\0') {
		/* loop */;
	}
	return dst;
}
