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
	fprintf(stderr, "usage: %s <input_file>\n", myprogname);
	exit(1);
}

#define	EVEN_SUFFIX	"_even.bin"
#define	ODD_SUFFIX	"_odd.bin"

int
main(int argc, char *argv[])
{
	char *fname, *even_fname, *odd_fname;
	FILE *infile, *even_file, *odd_file;
	char *ext;
	size_t fname_baselen;
	size_t count;
	int ch;

	set_myprogname(argv[0]);

	if (argc != 2) {
		usage();
	}
	fname = argv[1];

	ext = strrchr(fname, '.');
	if (ext == NULL) {
		fname_baselen = strlen(fname);
	} else {
		fname_baselen = ext - fname;
	}

	even_fname = malloc(fname_baselen + sizeof(EVEN_SUFFIX));
	if (even_fname == NULL) {
		fprintf(stderr, "unable to allocate even file name\n");
		exit(1);
	}
	memcpy(even_fname, fname, fname_baselen);
	strcat(even_fname, EVEN_SUFFIX);

	odd_fname = malloc(fname_baselen + sizeof(ODD_SUFFIX));
	if (odd_fname == NULL) {
		fprintf(stderr, "unable to allocate odd file name\n");
		exit(1);
	}
	memcpy(odd_fname, fname, fname_baselen);
	strcat(odd_fname, ODD_SUFFIX);

	infile = fopen(fname, "rb");
	if (infile == NULL) {
		fprintf(stderr, "unable to open input file '%s'\n", fname);
		exit(1);
	}

	even_file = fopen(even_fname, "w");
	if (even_file == NULL) {
		fprintf(stderr, "unable to create even output file '%s'\n",
		    even_fname);
		exit(1);
	}

	odd_file = fopen(odd_fname, "w");
	if (odd_file == NULL) {
		fprintf(stderr, "unable to create odd output file '%s'\n",
		    odd_fname);
		exit(1);
	}

	printf("Creating '%s' and '%s' from '%s'\n",
	    even_fname, odd_fname, fname);

	for (count = 0; (ch = fgetc(infile)) != EOF; count++) {
		fputc(ch, (count & 1) ? odd_file : even_file);
	}

	(void) fclose(even_file);
	(void) fclose(odd_file);
	(void) fclose(infile);

	printf("Wrote %zu bytes.\n", count);

	exit(0);
}
