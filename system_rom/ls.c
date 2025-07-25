/* From NetBSD: ls.c,v 1.5 2014/03/20 03:13:18 christos Exp */

/*-
 * Copyright (c) 2011
 *      The NetBSD Foundation, Inc. All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Martin Husemann.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
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
 */

/*
 * Copyright (c) 1996
 *	Matthias Drochner.  All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "syslib.h"
#include "sysfile.h"

#include "ls.h"

void
ls(const char *path)
{
	int             fd;
	struct stat     sb;
	size_t          size;
	const char	*fname = 0;
	char		*p;
	struct open_file *f;

	if ((fd = open(path, 0)) < 0
	    || fstat(fd, &sb) < 0
	    || (sb.st_mode & S_IFMT) != S_IFDIR) {
		/* Path supplied isn't a directory, open parent
		   directory and list matching files. */
		if (fd >= 0)
			close(fd);
		fname = strrchr(path, '/');
		if (fname) {
			size = fname - path;
			fname++;
			p = malloc(size + 1);
			if (!p)
				goto out;
			memcpy(p, path, size);
			p[size] = 0;
			fd = open(p, 0);
			free(p);
		} else {
			/* XXX FIXME XXX */
			fd = -1;
			errno = ENOTDIR;
		}

		if (fd < 0) {
			printf("ls: %s\n", strerror(errno));
			return;
		}
		if (fstat(fd, &sb) < 0) {
			printf("stat: %s\n", strerror(errno));
			goto out;
		}
		if ((sb.st_mode & S_IFMT) != S_IFDIR) {
			printf("%s: %s\n", path, strerror(ENOTDIR));
			goto out;
		}
	}

	if ((f = getfile(fd)) == NULL) {
		/* getfile() sets errno */
		goto out;
	}

	/* operation not defined on raw devices */
	if (f->f_flags & F_RAW) {
		errno = EOPNOTSUPP;
		goto out;
	}

	if (FS_LS(f->f_ops) != NULL)
		FS_LS(f->f_ops)(f, fname);
	else
		printf("no ls support for this file system\n");

out:
	close(fd);
}

struct lsentry {
	struct lsentry *e_next;
	uint32_t e_ino;
	const char *e_type;
	char	e_name[1];
};

void
lsadd(lsentry_t **names, const char *pattern, const char *name, size_t namelen,
    uint32_t ino, const char *type)
{
	lsentry_t *n, **np;

	if (pattern && !fnmatch(name, pattern))
		return;

	n = malloc(sizeof *n + namelen);
	if (!n) {
		printf("%d: %.*s (%s)\n", ino, (int)namelen, name, type);
		return;
	}

	n->e_ino = ino;
	n->e_type = type;
	memcpy(n->e_name, name, namelen);
	n->e_name[namelen] = '\0';

	for (np = names; *np; np = &(*np)->e_next) {
		if (strcmp(n->e_name, (*np)->e_name) < 0)
			break;
	}
	n->e_next = *np;
	*np = n;
}

void
lsprint(lsentry_t *names)
{
	if (!names) {
		printf("not found\n");
		return;
	}
	do {
		lsentry_t *n = names;
		printf("%d: %s (%s)\n", n->e_ino, n->e_name, n->e_type);
		names = n->e_next;
	} while (names);
}

void
lsfree(lsentry_t *names)
{
	if (!names)
		return;
	do {
		lsentry_t *n = names;
		names = n->e_next;
		free(n);
	} while (names);
}

void
lsunsup(const char *name)
{
	printf("The ls command is not currently supported for %s\n", name);
}
