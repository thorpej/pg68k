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

#include "systypes.h"
#include "syslib.h"
#include "sysfile.h"
#include "clock.h"

#include "img.h"

#define	IMG_MAXNAME	256
#define	IMG_BLKSIZE	512

struct image {
	uint64_t	img_nblks;
	int		img_fd;
	char		img_fname[IMG_MAXNAME];
};

static struct image image;

static int
img_strategy(struct open_file *f, int flags, daddr_t blk, size_t len,
    void *buf, size_t *actualp)
{
	struct image *img = f->f_devdata;
	size_t blkcnt;
	ssize_t actual = 0;
	int error = 0;

	*actualp = 0;

	if (len % IMG_BLKSIZE) {
		return EINVAL;
	}
	blkcnt = len / IMG_BLKSIZE;

	if (blk >= img->img_nblks) {
		return EIO;
	}

	if ((blk + blkcnt) > img->img_nblks) {
		blkcnt -= (blk + blkcnt) - img->img_nblks;
	}

	if (flags == 1/*XXX FREAD*/) {
		actual = read(img->img_fd, buf, blkcnt * IMG_BLKSIZE);
	} else {
		return EROFS;
	}

	if (actual == -1) {
		return EIO;
	}

	*actualp = actual;
	return error;
}

static int
img_open(struct open_file *f)
{
	struct image *img;

	img = &image;
	if (img->img_nblks == 0) {
		return ENXIO;
	}

	f->f_devdata = img;
	f->f_devaddr = 0;

	return 0;
}

static int
img_close(struct open_file *f)
{
	f->f_devdata = NULL;
	return 0;
}

static int
img_ioctl(struct open_file *f, u_long cmd, void *data)
{
	return EINVAL;
}

const struct devsw img_devsw = {
	.dv_name	=	"img",
	.dv_nargs	=	0,
	.dv_flags	=	DEV_F_NOPART,
	.dv_blk = {
		.dv_strategy =	img_strategy,
	},
	.dv_open	=	img_open,
	.dv_close	=	img_close,
	.dv_ioctl	=	img_ioctl,
};

void
img_attach(const char *fname)
{
	struct image *img = &image;
	struct stat sb;
	char dstr[DEV_STRING_SIZE];
	int fd = -1;

	if (img->img_nblks != 0) {
		printf("Image \"%s\" already attached.\n", img->img_fname);
		return;
	}

	fd = open(fname, O_RDONLY);
	if (fd < 0 || fstat(fd, &sb)) {
		printf("%s: %s\n", fname, strerror(errno));
		goto bad;
	}

	if (sb.st_size < IMG_BLKSIZE) {
		printf("%s: implausible image file\n", fname);
		goto bad;
	}

	img->img_fd = fd;
	img->img_nblks = sb.st_size / IMG_BLKSIZE;
	snprintf(img->img_fname, sizeof(img->img_fname), "%s%s",
	    dev_string(getfile(fd), dstr, sizeof(dstr)),
	    file_name(fd));

	printf("Image \"%s\" attached.\n", img->img_fname);
 	return;
 bad:
	if (fd >= 0) {
		close(fd);
	}
}

void
img_detach(void)
{
	struct image *img = &image;

	if (img->img_nblks == 0) {
		return;
	}

	printf("Image \"%s\" detached.\n", img->img_fname);
	close(img->img_fd);
	img->img_nblks = 0;
	img->img_fname[0] = '\0';
}
