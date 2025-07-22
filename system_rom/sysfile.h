/*	From NetBSD: stand.h,v 1.87 2022/04/30 09:24:05 rin Exp	*/

/*
 * Copyright (c) 1999 Christopher G. Demetriou.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Christopher G. Demetriou
 *      for the NetBSD Project.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
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

/*-
 * Copyright (c) 1993
 *      The Regents of the University of California.  All rights reserved.
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
 *      @(#)stand.h     8.1 (Berkeley) 6/11/93
 */

#ifndef sysfile_h_included
#define	sysfile_h_included

#include "config.h"
#include "systypes.h"

#ifdef CONFIG_MACH_HOST_SIM
#define	open		libsa_open
#define	close		libsa_close
#define	closeall	libsa_closeall
#define	read		libsa_read
#define	write		libsa_write
#define	lseek		libsa_lseek
#define	ioctl		libsa_ioctl
#define	stat		libsa_stat
#define	fstat		libsa_fstat
#endif /* CONFIG_MACH_HOST_SIM */

struct devsw;
struct fs_ops;
struct stat;

struct open_file {
	int		f_flags;	/* see F_* below */
	const struct devsw *f_dev;	/* pointer to device operations */
	void		*f_devdata;	/* device specific data */
	const struct fs_ops *f_ops;	/* pointer to file system operations */
	void		*f_fsdata;	/* file system specific data */
	off_t		f_offset;	/* current file offset (F_RAW) */
};

#define	SOPEN_MAX	4		/* # of concurrent open files */

/* f_flags values */
#define	F_READ		0x0001		/* file opened for reading */
#define	F_WRITE		0x0002		/* file opened for writing */
#define	F_RAW		0x0004		/* raw device open - no file system */
#define	F_NODEV		0x0008		/* network open - no device */

struct devsw {
	const char *dv_name;
	int	dv_nargs;
	int	(*dv_strategy)(void *, int, daddr_t, size_t, void *, size_t *);
	int	(*dv_open)(struct open_file *, ...);
	int	(*dv_close)(struct open_file *);
	int	(*dv_ioctl)(struct open_file *, u_long, void *);
};

extern const struct devsw *devsw[];	/* device array */
extern const int ndevs;			/* number of elements in devsw[] */

#define	DEV_NAME(d)		((d)->dv_name)
#define	DEV_STRATEGY(d)		((d)->dv_strategy)
#define	DEV_OPEN(d)		((d)->dv_open)
#define	DEV_CLOSE(d)		((d)->dv_close)
#define	DEV_IOCTL(d)		((d)->dv_ioctl)

struct fs_ops {
	int	(*fs_open)(const char *, struct open_file *);
	int	(*fs_close)(struct open_file *);
	int	(*fs_read)(struct open_file *, void *, size_t, size_t *);
	int	(*fs_write)(struct open_file *, void *, size_t size, size_t *);
	off_t	(*fs_seek)(struct open_file *, off_t, int);
	int	(*fs_stat)(struct open_file *, struct stat *);
	void	(*fs_ls)(struct open_file *, const char *);
};

extern const struct fs_ops *file_systems[];
extern const int nfsys;

#define	FS_OPEN(fs)		((fs)->fs_open)
#define	FS_CLOSE(fs)		((fs)->fs_close)
#define	FS_READ(fs)		((fs)->fs_read)
#define	FS_WRITE(fs)		((fs)->fs_write)
#define	FS_SEEK(fs)		((fs)->fs_seek)
#define	FS_STAT(fs)		((fs)->fs_stat)
#define	FS_LS(fs)		((fs)->fs_ls)

/* flag values for open(2) */
#define	O_RDONLY	0x0000
#define	O_WRONLY	0x0001
#define	O_RDWR		0x0002

/* whence values for lseek(2) */
#define	SEEK_SET	0	/* set file offset to offset */
#define	SEEK_CUR	1	/* set file offset to current plus offset */
#define	SEEK_END	2	/* set file offset to EOF plus offset */

struct stat {
	ino_t		st_ino;		/* inode's number */
	mode_t		st_mode;	/* inode protection mode */
	nlink_t		st_nlink;	/* number of hard links */
	uid_t		st_uid;		/* user ID of the file's owner */
	gid_t		st_gid;		/* group ID of the file's group */
	struct timespec	st_atim;	/* time of last access */
	struct timespec	st_mtim;	/* time of last data modification */
	struct timespec	st_ctim;	/* time of last file status change */
	struct timespec	st_birthtim;	/* time of creation */
	off_t		st_size;	/* file size, in bytes */
	uint64_t	st_blocks;	/* blocks allocated for file */
	uint32_t	st_blksize;	/* optimal blocksize for I/O */
	uint32_t	st_flags;	/* user defined flags for file */
	uint32_t	st_gen;		/* file generation number */
};

#define	S_IFMT		0170000	/* type of file mask */
#define	S_IFIFO		0010000	/* named pipe (fifo) */
#define	S_IFCHR		0020000	/* character special */
#define	S_IFDIR		0040000	/* directory */
#define	S_IFBLK		0060000	/* block special */
#define	S_IFREG		0100000	/* regular */
#define	S_IFLNK		0120000	/* symbolic link */
#define	S_ISVTX		0001000	/* save swapped text even after use */
#define	S_IFSOCK	0140000	/* socket */
#define	S_IFWHT		0160000	/* whiteout */
#define	S_ARCH1		0200000	/* Archive state 1, ls -l shows 'a' */
#define	S_ARCH2		0400000	/* Archive state 2, ls -l shows 'A' */

int	devopen(struct open_file *, const char *, const char **);
size_t	getsecsize(struct open_file *);
int	getfile(int, struct open_file **);
int	fnmatch(const char *, const char *);

int	open(const char *, int);
int	close(int);
void	closeall(void);
ssize_t	read(int, void *, size_t);
ssize_t	write(int, const void *, size_t);
off_t	lseek(int, off_t, int);
int	ioctl(int, u_long, void *);
int	stat(const char *, struct stat *);
int	fstat(int, struct stat *);
void	ls(const char *);

#endif /* sysfile_h_included */
