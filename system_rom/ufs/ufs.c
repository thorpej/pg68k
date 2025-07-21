/*	From NetBSD: ufs.c,v 1.88 2022/12/01 18:06:09 christos Exp	*/

/*-
 * Copyright (c) 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * The Mach Operating System project at Carnegie-Mellon University.
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
 *
 *
 * Copyright (c) 1990, 1991 Carnegie Mellon University
 * All Rights Reserved.
 *
 * Author: David Golub
 *
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

/*
 *	Stand-alone file reading package for UFS filesystems.
 */

#include "syslib.h"
#include "sysfile.h"

#include "ufs_dinode.h"
#include "ufs_dir.h"
#include "ffs_fs.h"
#include "ufs.h"

#include "endian.h"

#define	COMPAT_UFS

union dinode {
	struct ufs1_dinode	di1;
	struct ufs2_dinode	di2;
};

union indblk {
	void *		buf;
	int32_t *	ind32;
	int64_t *	ind64;
};

union inoblk {
	void *		buf;
	struct ufs1_dinode *ino1;
	struct ufs2_dinode *ino2;
};

typedef uint32_t	ino32_t;

#ifndef FSBTODB
#define FSBTODB(fs, indp) FFS_FSBTODB(fs, indp)
#endif

#define	MAXPATHLEN	PATH_MAX
#define	MAXBSIZE	(64 * 1024)

/*
 * To avoid having a lot of filesystem-block sized buffers lurking (which
 * could be 32k) we only keep a few entries of the indirect block map.
 * With 8k blocks, 2^8 blocks is ~500k so we reread the indirect block
 * ~13 times pulling in a 6M kernel.
 * The cache size must be smaller than the smallest filesystem block,
 * so LN2_IND_CACHE_SZ <= 9 (UFS2 and 4k blocks).
 */
#define LN2_IND_CACHE_SZ	6
#define IND_CACHE_SZ		(1 << LN2_IND_CACHE_SZ)
#define IND_CACHE_MASK		(IND_CACHE_SZ - 1)

/*
 * In-core open file.
 */
struct file {
	off_t		f_seekp;	/* seek pointer */
	struct fs	*f_fs;		/* pointer to super-block */
	union dinode	f_di;		/* copy of on-disk inode */
	u_int		f_nishift;	/* for blocks in indirect block */
	int64_t		f_ind_cache_block;
	int64_t		f_ind_cache[IND_CACHE_SZ];

	char		*f_buf;		/* buffer for data block */
	size_t		f_buf_size;	/* size of data block */
	daddr_t		f_buf_blkno;	/* block number of data block */
	bool		f_swapped;	/* FFS is other endian */
	bool		f_ufs2;
};

static int read_inode(ino32_t, struct open_file *);
static int block_map(struct open_file *, int64_t, int64_t *);
static int buf_read_file(struct open_file *, char **, size_t *);
static int search_directory(const char *, int, struct open_file *, ino32_t *);
static void ffs_oldfscompat(struct fs *);

static int64_t
ufs_get_indblkno(struct file *fp, union indblk *indp, size_t idx)
{
	if (fp->f_swapped) {
		return fp->f_ufs2 ? (int64_t)bswap64(indp->ind64[idx])
				  : (int32_t)bswap32(indp->ind32[idx]);
	}
	return fp->f_ufs2 ? indp->ind64[idx]
			  : indp->ind32[idx];
}

static uint64_t
ufs_get_size(struct file *fp)
{
	return fp->f_ufs2 ? fp->f_di.di2.di_size : fp->f_di.di1.di_size;
}

static uint16_t
ufs_get_mode(struct file *fp)
{
	return fp->f_ufs2 ? fp->f_di.di2.di_mode : fp->f_di.di1.di_mode;
}

static uint32_t
ufs_get_uid(struct file *fp)
{
	return fp->f_ufs2 ? fp->f_di.di2.di_uid : fp->f_di.di1.di_uid;
}

static uint32_t
ufs_get_gid(struct file *fp)
{
	return fp->f_ufs2 ? fp->f_di.di2.di_gid : fp->f_di.di1.di_gid;
}

static int64_t
ufs_get_db(struct file *fp, int64_t blk)
{
	return fp->f_ufs2 ? fp->f_di.di2.di_db[blk] : fp->f_di.di1.di_db[blk];
}

static int64_t
ufs_get_ib(struct file *fp, int64_t blk)
{
	return fp->f_ufs2 ? fp->f_di.di2.di_ib[blk] : fp->f_di.di1.di_ib[blk];
}

static char *
ufs_get_symlinkbuf(struct file *fp)
{
	return fp->f_ufs2 ? (char *)fp->f_di.di2.di_db
			  : (char *)fp->f_di.di1.di_db;
}

static int
ufs_probe(struct file *fp, struct fs *fs, daddr_t blkno)
{
	uint32_t magic = fs->fs_magic;

	for (fp->f_swapped = fp->f_ufs2 = false;;) {
		switch (magic) {
		case FS_UFS1_MAGIC:
			if (blkno == SBLOCK_UFS2) {
				/*
				 * See comment in ffs_fs.h above SBLOCKSEARCH.
				 */
				return EINVAL;
			}
			return 0;

		case FS_UFS2_MAGIC:
		case FS_UFS2EA_MAGIC:
			fp->f_ufs2 = true;
			return 0;

		default:
			if (fp->f_swapped) {
				return EINVAL;
			}
			magic = bswap32(magic);
			fp->f_swapped = true;
		}
	}
	/* NOTREACHED */
}

static inline uint16_t
ffs_get_reclen(struct file *fp, struct direct *dp)
{
	return fp->f_swapped ? bswap16(dp->d_reclen) : dp->d_reclen;
}

static inline uint32_t
ffs_get_ino(struct file *fp, struct direct *dp)
{
	return fp->f_swapped ? bswap32(dp->d_ino) : dp->d_ino;
}

static int	ufs_close(struct open_file *);

/*
 * Read a new inode into a file structure.
 */
static int
read_inode(ino32_t inumber, struct open_file *f)
{
	struct file *fp = (struct file *)f->f_fsdata;
	struct fs *fs = fp->f_fs;
	size_t rsize;
	int rc;
	daddr_t inode_sector = 0; /* XXX: gcc */
	union inoblk inoblk = {
		.buf = fp->f_buf,
	};

	inode_sector = FSBTODB(fs, ino_to_fsba(fs, inumber));

	/*
	 * Read inode and save it.
	 */
	rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
	    inode_sector, fs->fs_bsize, inoblk.buf, &rsize);
	if (rc)
		return rc;
	if (rsize != (size_t)fs->fs_bsize)
		return EIO;

	if (fp->f_ufs2) {
		fp->f_di.di2 = inoblk.ino2[ino_to_fsbo(fs, inumber)];
		if (fp->f_swapped) {
			ffs_dinode2_swap(&fp->f_di.di2, &fp->f_di.di2);
		}
	} else {
		fp->f_di.di1 = inoblk.ino1[ino_to_fsbo(fs, inumber)];
		if (fp->f_swapped) {
			ffs_dinode1_swap(&fp->f_di.di1, &fp->f_di.di1);
		}
	}

	/*
	 * Clear out the old buffers
	 */
	fp->f_ind_cache_block = -1;
	fp->f_buf_blkno = -1;
	return rc;
}

/*
 * Given an offset in a file, find the disk block number that
 * contains that block.
 */
static int
block_map(struct open_file *f, int64_t file_block, int64_t *disk_block_p)
{
	struct file *fp = (struct file *)f->f_fsdata;
	struct fs *fs = fp->f_fs;
	u_int level;
	int64_t ind_cache;
	int64_t ind_block_num;
	size_t rsize;
	int rc;
	union indblk indblk = {
		.buf = fp->f_buf,
	};

	/*
	 * Index structure of an inode:
	 *
	 * di_db[0..UFS_NDADDR-1]	hold block numbers for blocks
	 *			0..UFS_NDADDR-1
	 *
	 * di_ib[0]		index block 0 is the single indirect block
	 *			holds block numbers for blocks
	 *			UFS_NDADDR .. UFS_NDADDR + UFS_NINDIR(fs)-1
	 *
	 * di_ib[1]		index block 1 is the double indirect block
	 *			holds block numbers for INDEX blocks for blocks
	 *			UFS_NDADDR + UFS_NINDIR(fs) ..
	 *			UFS_NDADDR + UFS_NINDIR(fs) + UFS_NINDIR(fs)**2 - 1
	 *
	 * di_ib[2]		index block 2 is the triple indirect block
	 *			holds block numbers for double-indirect
	 *			blocks for blocks
	 *			UFS_NDADDR + UFS_NINDIR(fs) + UFS_NINDIR(fs)**2 ..
	 *			UFS_NDADDR + UFS_NINDIR(fs) + UFS_NINDIR(fs)**2
	 *				+ UFS_NINDIR(fs)**3 - 1
	 */


	if (file_block < UFS_NDADDR) {
		/* Direct block. */
		*disk_block_p = ufs_get_db(fp, file_block);
		return 0;
	}

	file_block -= UFS_NDADDR;

	ind_cache = file_block >> LN2_IND_CACHE_SZ;
	if (ind_cache == fp->f_ind_cache_block) {
		*disk_block_p = fp->f_ind_cache[file_block & IND_CACHE_MASK];
		return 0;
	}

	for (level = 0;;) {
		level += fp->f_nishift;
		if (file_block < (int64_t)1 << level)
			break;
		if (level > UFS_NIADDR * fp->f_nishift)
			/* Block number too high */
			return EFBIG;
		file_block -= (int64_t)1 << level;
	}

	ind_block_num = ufs_get_ib(fp, level / fp->f_nishift - 1);

	for (;;) {
		level -= fp->f_nishift;
		if (ind_block_num == 0) {
			*disk_block_p = 0;	/* missing */
			return 0;
		}

		/*
		 * If we were feeling brave, we could work out the number
		 * of the disk sector and read a single disk sector instead
		 * of a filesystem block.
		 * However we don't do this very often anyway...
		 */
		rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
			FSBTODB(fp->f_fs, ind_block_num), fs->fs_bsize,
			indblk.buf, &rsize);
		if (rc)
			return rc;
		if (rsize != (size_t)fs->fs_bsize)
			return EIO;
		ind_block_num = ufs_get_indblkno(fp, &indblk,
		    file_block >> level);
		if (level == 0)
			break;
		file_block &= (1 << level) - 1;
	}


	/* Save the part of the block that contains this sector */
	for (size_t i = 0; i < IND_CACHE_SZ; i++) {
		fp->f_ind_cache[i] = ufs_get_indblkno(fp, &indblk,
		    (file_block & ~IND_CACHE_MASK) + i);
	}
	fp->f_ind_cache_block = ind_cache;

	*disk_block_p = ind_block_num;

	return 0;
}

/*
 * Read a portion of a file into an internal buffer.
 * Return the location in the buffer and the amount in the buffer.
 */
static int
buf_read_file(struct open_file *f, char **buf_p, size_t *size_p)
{
	struct file *fp = (struct file *)f->f_fsdata;
	struct fs *fs = fp->f_fs;
	long off;
	uint64_t file_size;
	int64_t file_block;
	size_t block_size;
	int rc;

	off = ffs_blkoff(fs, fp->f_seekp);
	file_block = ffs_lblkno(fs, fp->f_seekp);
	file_size = ufs_get_size(fp);
	block_size = (size_t)ffs_sblksize(fs, (int64_t)file_size, file_block);

	if (file_block != fp->f_buf_blkno) {
		int64_t disk_block = 0; /* XXX: gcc */
		rc = block_map(f, file_block, &disk_block);
		if (rc)
			return rc;

		if (disk_block == 0) {
			memset(fp->f_buf, 0, block_size);
			fp->f_buf_size = block_size;
		} else {
			rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
				FSBTODB(fs, disk_block),
				block_size, fp->f_buf, &fp->f_buf_size);
			if (rc)
				return rc;
		}

		fp->f_buf_blkno = file_block;
	}

	/*
	 * Return address of byte in buffer corresponding to
	 * offset, and size of remainder of buffer after that
	 * byte.
	 */
	*buf_p = fp->f_buf + off;
	*size_p = block_size - off;

	/*
	 * But truncate buffer at end of file.
	 */
	if (*size_p > file_size - fp->f_seekp)
		*size_p = file_size - fp->f_seekp;

	return 0;
}

/*
 * Search a directory for a name and return its
 * inode number.
 */
static int
search_directory(const char *name, int length, struct open_file *f,
	ino32_t *inumber_p)
{
	struct file *fp = (struct file *)f->f_fsdata;
	struct direct *dp;
	struct direct *edp;
	char *buf;
	size_t buf_size;
	uint64_t file_size;
	int namlen;
	int rc;

	file_size = ufs_get_size(fp);

	fp->f_seekp = 0;
	while (fp->f_seekp < (off_t)file_size) {
		rc = buf_read_file(f, &buf, &buf_size);
		if (rc)
			return rc;

		dp = (struct direct *)buf;
		edp = (struct direct *)(buf + buf_size);
		for (; dp < edp;
		     dp = (void *)((char *)dp + ffs_get_reclen(fp, dp))) {
			if (ffs_get_reclen(fp, dp) <= 0)
				break;
			if (ffs_get_ino(fp, dp) == (ino32_t)0)
				continue;
#if _BYTE_ORDER == _LITTLE_ENDIAN
			if (fp->f_fs->fs_maxsymlinklen <= 0)
				namlen = dp->d_type;
			else
#endif
				namlen = dp->d_namlen;
			if (namlen == length &&
			    !memcmp(name, dp->d_name, length)) {
				/* found entry */
				*inumber_p = ffs_get_ino(fp, dp);
				return 0;
			}
		}
		fp->f_seekp += buf_size;
	}
	return ENOENT;
}

static int
ffs_find_superblock(struct open_file *f, struct fs *fs)
{
	static const daddr_t sblock_try[] = SBLOCKSEARCH;
	struct file *fp = (struct file *)f->f_fsdata;
	size_t buf_size;
	int rc, i;

	for (i = 0; sblock_try[i] != -1; i++) {
		rc = DEV_STRATEGY(f->f_dev)(f->f_devdata, F_READ,
		    sblock_try[i] / getsecsize(f), SBLOCKSIZE, fs, &buf_size);
		if (rc)
			return rc;
		if (buf_size != SBLOCKSIZE)
			return EINVAL;
		rc = ufs_probe(fp, fs, sblock_try[i]);
		if (rc == 0) {
			if (fp->f_swapped) {
				ffs_sb_swap(fs, fs);
			}
			if (fs->fs_sblockloc != sblock_try[i]) {
				/* an alternate superblock - try again */
				continue;
			}
			/* Found it! */
			return 0;
		}
	}
	return EINVAL;
}

/*
 * Open a file.
 */
static int
ufs_open(const char *path, struct open_file *f)
{
	const char *cp, *ncp;
	int c;
	ino32_t inumber;
	struct file *fp;
	struct fs *fs;
	int rc;
	ino32_t parent_inumber;
	int nlinks = 0;
	char namebuf[MAXPATHLEN+1];
	char *buf;

	/* allocate file system specific data structure */
	fp = malloc(sizeof(struct file));
	memset(fp, 0, sizeof(struct file));
	f->f_fsdata = (void *)fp;

	/* allocate space and read super block */
	fs = malloc(SBLOCKSIZE);
	fp->f_fs = fs;

	rc = ffs_find_superblock(f, fs);
	if (rc)
		goto out;

	ffs_oldfscompat(fs);

	if (fs->fs_bsize > MAXBSIZE ||
	    (size_t)fs->fs_bsize < sizeof(struct fs)) {
		rc = EINVAL;
		goto out;
	}

	/*
	 * Calculate indirect block levels.
	 */
	{
		int64_t mult;
		int ln2;

		/*
		 * We note that the number of indirect blocks is always
		 * a power of 2.  This lets us use shifts and masks instead
		 * of divide and remainder and avoids pulling in the
		 * 64bit division routine into the boot code.
		 */
		mult = FFS_NINDIR(fs);
#ifdef DEBUG
		if (mult & (mult - 1)) {
			/* Hummm was't a power of 2 */
			rc = EINVAL;
			goto out;
		}
#endif
		for (ln2 = 0; mult != 1; ln2++)
			mult >>= 1;

		fp->f_nishift = ln2;
	}

	/* alloc a block sized buffer used for all fs transfers */
	fp->f_buf = malloc(fs->fs_bsize);
	inumber = UFS_ROOTINO;
	if ((rc = read_inode(inumber, f)) != 0)
		goto out;

	cp = path;
	while (*cp) {
		uint16_t file_mode;

		/*
		 * Remove extra separators
		 */
		while (*cp == '/')
			cp++;
		if (*cp == '\0')
			break;

		file_mode = ufs_get_mode(fp);

		/*
		 * Check that current node is a directory.
		 */
		if ((file_mode & IFMT) != IFDIR) {
			rc = ENOTDIR;
			goto out;
		}

		/*
		 * Get next component of path name.
		 */
		ncp = cp;
		while ((c = *cp) != '\0' && c != '/')
			cp++;

		/*
		 * Look up component in current directory.
		 * Save directory inumber in case we find a
		 * symbolic link.
		 */
		parent_inumber = inumber;
		rc = search_directory(ncp, cp - ncp, f, &inumber);
		if (rc)
			goto out;

		/*
		 * Open next component.
		 */
		if ((rc = read_inode(inumber, f)) != 0)
			goto out;

		/*
		 * Check for symbolic link.
		 */
		if ((file_mode & IFMT) == IFLNK) {
			int link_len = (int)ufs_get_size(fp);
			int len;

			len = strlen(cp);

			if (link_len + len > MAXPATHLEN ||
			    ++nlinks > MAXSYMLINKS) {
				rc = ENOENT;
				goto out;
			}

			memmove(&namebuf[link_len], cp, len + 1);

			if (link_len < fs->fs_maxsymlinklen) {
				memcpy(namebuf,
				    ufs_get_symlinkbuf(fp), link_len);
			} else {
				/*
				 * Read file for symbolic link
				 */
				size_t buf_size;
				int64_t disk_block;

				buf = fp->f_buf;
				rc = block_map(f, 0, &disk_block);
				if (rc)
					goto out;

				rc = DEV_STRATEGY(f->f_dev)(f->f_devdata,
					F_READ, FSBTODB(fs, disk_block),
					fs->fs_bsize, buf, &buf_size);
				if (rc)
					goto out;

				memcpy(namebuf, buf, link_len);
			}

			/*
			 * If relative pathname, restart at parent directory.
			 * If absolute pathname, restart at root.
			 */
			cp = namebuf;
			if (*cp != '/')
				inumber = parent_inumber;
			else
				inumber = (ino32_t)UFS_ROOTINO;

			if ((rc = read_inode(inumber, f)) != 0)
				goto out;
		}
	}

	/*
	 * Found terminal component.
	 */
	rc = 0;

	fp->f_seekp = 0;		/* reset seek pointer */

out:
	if (rc)
		ufs_close(f);
	return rc;
}

static int
ufs_close(struct open_file *f)
{
	struct file *fp = (struct file *)f->f_fsdata;

	f->f_fsdata = NULL;
	if (fp == NULL)
		return 0;

	if (fp->f_buf)
		free(fp->f_buf);
	free(fp->f_fs);
	free(fp);
	return 0;
}

/*
 * Copy a portion of a file into kernel memory.
 * Cross block boundaries when necessary.
 */
static int
ufs_read(struct open_file *f, void *start, size_t size, size_t *resid)
{
	struct file *fp = (struct file *)f->f_fsdata;
	size_t csize;
	char *buf;
	size_t buf_size;
	int rc = 0;
	char *addr = start;

	while (size != 0) {
		if (fp->f_seekp >= (off_t)ufs_get_size(fp))
			break;

		rc = buf_read_file(f, &buf, &buf_size);
		if (rc)
			break;

		csize = size;
		if (csize > buf_size)
			csize = buf_size;

		memcpy(addr, buf, csize);

		fp->f_seekp += csize;
		addr += csize;
		size -= csize;
	}
	if (resid)
		*resid = size;
	return rc;
}

/*
 * Not implemented.
 */
static int
ufs_write(struct open_file *f, void *start, size_t size, size_t *resid)
{

	return EROFS;
}

static off_t
ufs_seek(struct open_file *f, off_t offset, int where)
{
	struct file *fp = (struct file *)f->f_fsdata;

	switch (where) {
	case SEEK_SET:
		fp->f_seekp = offset;
		break;
	case SEEK_CUR:
		fp->f_seekp += offset;
		break;
	case SEEK_END:
		fp->f_seekp = ufs_get_size(fp) - offset;
		break;
	default:
		return -1;
	}
	return fp->f_seekp;
}

static int
ufs_stat(struct open_file *f, struct stat *sb)
{
	struct file *fp = (struct file *)f->f_fsdata;

	/* only important stuff */
	memset(sb, 0, sizeof *sb);
	sb->st_mode = ufs_get_mode(fp);
	sb->st_uid = ufs_get_uid(fp);
	sb->st_gid = ufs_get_gid(fp);
	sb->st_size = ufs_get_size(fp);
	return 0;
}

#include "ls.h"

static const char    *const typestr[] = {
	"unknown",
	"FIFO",
	"CHR",
	0,
	"DIR",
	0,
	"BLK",
	0,
	"REG",
	0,
	"LNK",
	0,
	"SOCK",
	0,
	"WHT"
};

static void
ufs_ls(struct open_file *f, const char *pattern)
{
	struct file *fp = (struct file *)f->f_fsdata;
	char *buf;
	size_t buf_size;
	lsentry_t *names = NULL;
	off_t file_size = ufs_get_size(fp);

	fp->f_seekp = 0;
	while (fp->f_seekp < file_size) {
		struct direct  *dp, *edp;
		int rc = buf_read_file(f, &buf, &buf_size);
		if (rc)
			goto out;
		/* some firmware might use block size larger than DEV_BSIZE */
		if (buf_size < UFS_DIRBLKSIZ)
			goto out;

		dp = (struct direct *)buf;
		edp = (struct direct *)(buf + buf_size);

		for (; dp < edp;
		     dp = (void *)((char *)dp + ffs_get_reclen(fp, dp))) {
			const char *t;
			if (ffs_get_ino(fp, dp) == 0)
				continue;

			if (dp->d_type >= NELEM(typestr) ||
			    !(t = typestr[dp->d_type])) {
				/*
				 * This does not handle "old"
				 * filesystems properly. On little
				 * endian machines, we get a bogus
				 * type name if the namlen matches a
				 * valid type identifier. We could
				 * check if we read namlen "0" and
				 * handle this case specially, if
				 * there were a pressing need...
				 */
				printf("bad dir entry\n");
				goto out;
			}
			lsadd(&names, pattern, dp->d_name, strlen(dp->d_name),
			    ffs_get_ino(fp, dp), t);
		}
		fp->f_seekp += buf_size;
	}
	lsprint(names);
out:	lsfree(names);
}

/*
 * Sanity checks for old file systems.
 *
 * XXX - goes away some day.
 * Stripped of stuff libsa doesn't need.....
 */
static void
ffs_oldfscompat(struct fs *fs)
{

#ifdef COMPAT_UFS
	/*
	 * Newer Solaris versions have a slightly incompatible
	 * superblock - so always calculate this values on the fly, which
	 * is good enough for libsa purposes
	 */
	if (fs->fs_magic == FS_UFS1_MAGIC
#ifndef COMPAT_SOLARIS_UFS
	    && fs->fs_old_inodefmt < FS_44INODEFMT
#endif
	    ) {
		fs->fs_qbmask = ~fs->fs_bmask;
		fs->fs_qfmask = ~fs->fs_fmask;
	}
#endif
}

const struct fs_ops ufs_fsops = {
	.fs_open	=	ufs_open,
	.fs_close	=	ufs_close,
	.fs_read	=	ufs_read,
	.fs_write	=	ufs_write,
	.fs_seek	=	ufs_seek,
	.fs_stat	=	ufs_stat,
	.fs_ls		=	ufs_ls,
};
