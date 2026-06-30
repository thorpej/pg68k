/*
 * Copyright (c) 2026 Jason R. Thorpe.
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

#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

static const char *myprogname;

static void
set_myprogname(const char *argv0)
{
	const char *cp = strrchr(argv0, '/');
	if (cp != NULL) {
		myprogname = cp + 1;
	} else {
		myprogname = argv0;
	}
}

static void
usage(void)
{
	fprintf(stderr, "usage: %s <sz> <input_file>\n", myprogname);
	exit(1);
}

int
main(int argc, char *argv[])
{
	char *fname, *cp;
	void *buf;
	FILE *fp;
	int mult = 1, count;
	long padsize, roundsize, fsize;

	set_myprogname(argv[0]);

	if (argc != 3) {
		usage();
	}

	padsize = strtol(argv[1], &cp, 0);
	if (padsize <= 0 || padsize == LONG_MAX) {
		goto bad_pad_size;
	}
	if (*cp == 'm' || *cp == 'M') {
		cp++;
		mult = 1024 * 1024;
	} else if (*cp == 'k' || *cp == 'K') {
		cp++;
		mult = 1024;
	}
	if (*cp != '\0') {
		goto bad_pad_size;
	}

	if (padsize * mult < padsize) {
 bad_pad_size:
		fprintf(stderr, "bad pad size: %s\n", argv[1]);
		exit(1);
	}
	padsize *= mult;

	if (((padsize - 1) & padsize) != 0) {
		fprintf(stderr, "pad size is not a power-of-2\n");
		exit(1);
	}

	fname = argv[2];
	fp = fopen(fname, "rb+");
	if (fp == NULL) {
		fprintf(stderr, "unable to open file '%s'\n", fname);
		exit(1);
	}

	fseek(fp, 0, SEEK_END);
	fsize = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	if (padsize < fsize) {
		fprintf(stderr, "pad size %ld less than file size %ld\n",
		    padsize, fsize);
		exit(1);
	}

	for (roundsize = padsize; (roundsize >> 1) >= fsize;) {
		/* Find smallest power-of-2 size that it fits in. */
		roundsize >>= 1;
	}
	printf("Rounding '%s' to %ld bytes.\n", fname, roundsize);

	buf = calloc(1, roundsize);
	if (fread(buf, 1, fsize, fp) != (size_t)fsize) {
		fprintf(stderr, "failed to read `%s'.\n", fname);
		exit(1);
	}

	fseek(fp, 0, SEEK_SET);
	for (count = 0; padsize != 0; count++, padsize -= roundsize) {
		if (fwrite(buf, 1, roundsize, fp) != (size_t)roundsize) {
			fprintf(stderr, "failed to write '%s'.\n", fname);
			exit(1);
		}
	}

	(void) fclose(fp);

	printf("Repeated image %d times.\n", count);

	exit(0);
}
