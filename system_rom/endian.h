/*	From NetBSD: endian.h,v 1.37 2025/04/04 04:37:32 rin Exp	*/

/*
 * Copyright (c) 1987, 1991, 1993
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
 *
 *	@(#)endian.h	8.1 (Berkeley) 6/11/93
 */

#ifndef endian_h_included
#define endian_h_included

/*
 * Definitions for byte order, according to byte significance from low
 * address to high.
 */
#define	_LITTLE_ENDIAN	1234	/* LSB first: i386, vax */
#define	_BIG_ENDIAN	4321	/* MSB first: 68000, ibm, net */
#define	_PDP_ENDIAN	3412	/* LSB first in word, MSW first in long */

__BEGIN_DECLS
uint32_t htonl(uint32_t) __constfunc;
uint16_t htons(uint16_t) __constfunc;
uint32_t ntohl(uint32_t) __constfunc;
uint16_t ntohs(uint16_t) __constfunc;
__END_DECLS

#define	_BYTE_ORDER	_BIG_ENDIAN

/*
 * Define the order of 32-bit words in 64-bit words.
 */
#if _BYTE_ORDER == _LITTLE_ENDIAN
#define _QUAD_HIGHWORD 1
#define _QUAD_LOWWORD 0
#endif

#if _BYTE_ORDER == _BIG_ENDIAN
#define _QUAD_HIGHWORD 0
#define _QUAD_LOWWORD 1
#endif

/*
 * Macros for network/external number representation conversion.
 */
#if _BYTE_ORDER == _BIG_ENDIAN
#define	ntohl(x)	__CAST(uint32_t, (x))
#define	ntohs(x)	__CAST(uint16_t, (x))
#define	htonl(x)	__CAST(uint32_t, (x))
#define	htons(x)	__CAST(uint16_t, (x))

#define	NTOHL(x)	__CAST(void, (x))
#define	NTOHS(x)	__CAST(void, (x))
#define	HTONL(x)	__CAST(void, (x))
#define	HTONS(x)	__CAST(void, (x))

#else	/* _LITTLE_ENDIAN */

#define	ntohl(x)	bswap32(__CAST(uint32_t, (x)))
#define	ntohs(x)	bswap16(__CAST(uint16_t, (x)))
#define	htonl(x)	bswap32(__CAST(uint32_t, (x)))
#define	htons(x)	bswap16(__CAST(uint16_t, (x)))

#define	NTOHL(x)	(x) = ntohl(__CAST(uint32_t, (x)))
#define	NTOHS(x)	(x) = ntohs(__CAST(uint16_t, (x)))
#define	HTONL(x)	(x) = htonl(__CAST(uint32_t, (x)))
#define	HTONS(x)	(x) = htons(__CAST(uint16_t, (x)))
#endif	/* _LITTLE_ENDIAN */

/*
 * Macros to convert to a specific endianness.
 */

#if _BYTE_ORDER == _BIG_ENDIAN

#define htobe16(x)	__CAST(uint16_t, (x))
#define htobe32(x)	__CAST(uint32_t, (x))
#define htobe64(x)	__CAST(uint64_t, (x))
#define htole16(x)	bswap16(__CAST(uint16_t, (x)))
#define htole32(x)	bswap32(__CAST(uint32_t, (x)))
#define htole64(x)	bswap64(__CAST(uint64_t, (x)))

#define HTOBE16(x)	__CAST(void, (x))
#define HTOBE32(x)	__CAST(void, (x))
#define HTOBE64(x)	__CAST(void, (x))
#define HTOLE16(x)	(x) = bswap16(__CAST(uint16_t, (x)))
#define HTOLE32(x)	(x) = bswap32(__CAST(uint32_t, (x)))
#define HTOLE64(x)	(x) = bswap64(__CAST(uint64_t, (x)))

#else	/* _LITTLE_ENDIAN */

#define htobe16(x)	bswap16(__CAST(uint16_t, (x)))
#define htobe32(x)	bswap32(__CAST(uint32_t, (x)))
#define htobe64(x)	bswap64(__CAST(uint64_t, (x)))
#define htole16(x)	__CAST(uint16_t, (x))
#define htole32(x)	__CAST(uint32_t, (x))
#define htole64(x)	__CAST(uint64_t, (x))

#define HTOBE16(x)	(x) = bswap16(__CAST(uint16_t, (x)))
#define HTOBE32(x)	(x) = bswap32(__CAST(uint32_t, (x)))
#define HTOBE64(x)	(x) = bswap64(__CAST(uint64_t, (x)))
#define HTOLE16(x)	__CAST(void, (x))
#define HTOLE32(x)	__CAST(void, (x))
#define HTOLE64(x)	__CAST(void, (x))

#endif	/* _LITTLE_ENDIAN */

#define be16toh(x)	htobe16(x)
#define be32toh(x)	htobe32(x)
#define be64toh(x)	htobe64(x)
#define le16toh(x)	htole16(x)
#define le32toh(x)	htole32(x)
#define le64toh(x)	htole64(x)

#define BE16TOH(x)	HTOBE16(x)
#define BE32TOH(x)	HTOBE32(x)
#define BE64TOH(x)	HTOBE64(x)
#define LE16TOH(x)	HTOLE16(x)
#define LE32TOH(x)	HTOLE32(x)
#define LE64TOH(x)	HTOLE64(x)

/*
 * Routines to encode/decode big- and little-endian multi-octet values
 * to/from an octet stream.
 */
#define __GEN_ENDIAN_ENC(bits, endian) \
static __inline void __unused \
endian ## bits ## enc(void *dst, uint ## bits ## _t u) \
{ \
	u = hto ## endian ## bits (u); \
	__builtin_memcpy(dst, &u, sizeof(u)); \
}

__GEN_ENDIAN_ENC(16, be)
__GEN_ENDIAN_ENC(32, be)
__GEN_ENDIAN_ENC(64, be)
__GEN_ENDIAN_ENC(16, le)
__GEN_ENDIAN_ENC(32, le)
__GEN_ENDIAN_ENC(64, le)
#undef __GEN_ENDIAN_ENC

#define __GEN_ENDIAN_DEC(bits, endian) \
static __inline uint ## bits ## _t __unused \
endian ## bits ## dec(const void *buf) \
{ \
	uint ## bits ## _t u; \
	__builtin_memcpy(&u, buf, sizeof(u)); \
	return endian ## bits ## toh (u); \
}

__GEN_ENDIAN_DEC(16, be)
__GEN_ENDIAN_DEC(32, be)
__GEN_ENDIAN_DEC(64, be)
__GEN_ENDIAN_DEC(16, le)
__GEN_ENDIAN_DEC(32, le)
__GEN_ENDIAN_DEC(64, le)
#undef __GEN_ENDIAN_DEC

#endif /* !endian_h_included */
