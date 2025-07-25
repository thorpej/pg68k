/*
 * char *strdup(const char *src)
 *
 * Written July 2025, Jason R. Thorpe.
 * Public domain.
 */

#include "syslib.h"

char *
strdup(const char *src)
{
	char *dst = malloc(strlen(src) + 1);
	if (dst != NULL) {
		strcpy(dst, src);
	}
	return dst;
}
