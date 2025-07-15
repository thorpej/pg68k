/*
 * int memcmp(const void *v1, const void *v2, size_t n)
 *
 * Written July 2025, Jason R. Thorpe.
 * Public domain.
 */

#include "syslib.h"

int
memcmp(const void *v1, const void *v2, size_t n)
{
	const char *cp1 = v1;
	const char *cp2 = v2;
	int rv;

	for (rv = 0; rv == 0 && n != 0; n--) {
		rv = *cp1++ - *cp2++;
	}
	return rv;
}
