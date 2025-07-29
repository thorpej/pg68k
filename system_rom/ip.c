/*	From NetBSD: ip.c,v 1.5 2022/07/08 07:02:47 skrll Exp	*/

/*
 * Copyright (c) 1992 Regents of the University of California.
 * Copyright (c) 2010 Zoltan Arnold NAGY
 * All rights reserved.
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
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
 *	This product includes software developed by the University of
 *	California, Lawrence Berkeley Laboratory and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
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

#include "sysfile.h"
#include "net.h"

/*
 * Definitions for internet protocol version 4.
 * Per RFC 791, September 1981.
 */
#define	IPVERSION	4

struct ip {
#if _BYTE_ORDER == _LITTLE_ENDIAN
	unsigned int ip_hl:4,		/* header length */
	             ip_v:4;		/* version */
#elif _BYTE_ORDER == _BIG_ENDIAN
	unsigned int ip_v:4,		/* version */
	             ip_hl:4;		/* header length */
#else
#error You lose, PDP-11.
#endif
	uint8_t  ip_tos;		/* type of service */
	uint16_t ip_len;		/* total length */
	uint16_t ip_id;			/* identification */
	uint16_t ip_off;		/* fragment offset field */
#define	IP_RF		0x8000		/* reserved fragment flag */
#define	IP_EF		0x8000		/* evil flag, per RFC 3514 */
#define	IP_DF		0x4000		/* dont fragment flag */
#define	IP_MF		0x2000		/* more fragments flag */
#define	IP_OFFMASK	0x1fff		/* mask for fragmenting bits */
	uint8_t  ip_ttl;		/* time to live */
	uint8_t  ip_p;			/* protocol */
	uint16_t ip_sum;		/* checksum */
	uint32_t ip_src;		/* source address */
	uint32_t ip_dst;		/* destination address */
};

#define	IP_MAXPACKET	65535		/* maximum packet size */
#define	IP_MINFRAGSIZE	69		/* minimum size that can be fraged */

#define	IPDEFTTL	64		/* default ttl, from RFC 1340 */
#define	IPTTLDEC	1		/* subtracted when forwarding */

static int
ip_send_internal(struct open_file *f, struct packet *pkt, struct ip *ip)
{
	const uint8_t *ea;

	if (ip->ip_dst == INADDR_BROADCAST || ip->ip_src == 0 ||
	    f->f_net.f_netmask == 0 ||
	    SAMENET(ip->ip_src, ip->ip_dst, f->f_net.f_netmask)) {
		ea = arp_whohas(f, ip->ip_dst);
	} else {
		ea = arp_whohas(f, f->f_net.f_gateway);
	}
	if (ea == NULL) {
		EHOSTUNREACH;
	}

	return ether_send(f, pkt, ea, ETHERTYPE_IP);
}

int
ip_send(struct open_file *f, struct packet *pkt, uint8_t proto)
{
	int error;

	struct ip ip = {
		.ip_v = IPVERSION,
		.ip_p = proto,
		.ip_ttl = IPDEFTTL,
		.ip_src = f->f_net.f_laddr,
		.ip_dst = f->f_net.f_raddr,
	};
	ip.ip_hl = sizeof(ip) >> 2;
	ip.ip_len = htons((uint16_t)pkt->pkt_len + sizeof(ip));
	ip.ip_sum = ip_cksum(&ip, sizeof(ip));

	error = packet_prepend(pkt, &ip, sizeof(ip));
	if (error == 0) {
		error = ip_send_internal(f, pkt, &ip);
	}
	return error;
}

int
ip_recv(struct open_file *f, struct packet *pkt, int tleft, uint8_t proto)
{
	struct ip *ip;
	size_t len;
	int error;
	uint16_t etype;

	error = ether_recv(f, pkt, tleft, &etype);
	if (error) {
		return error;
	}
	if (etype == ETHERTYPE_ARP) {
		arp_reply(f, pkt);
		return -1;
	}
	if (etype != ETHERTYPE_IP) {
		return -1;
	}
	error = packet_get_layer(pkt, (void **)&ip, &len);
	if (error) {
		return error;
	}
	if (ip->ip_v != IPVERSION || ip->ip_p != proto) {
		return -1;
	}
	size_t hlen = ip->ip_hl << 2;
	if (hlen != sizeof(*ip) || ip_cksum(ip, hlen) != 0) {
		return -1;
	}
	if (ntohs(ip->ip_len) < hlen) {
		return -1;
	}
	if (f->f_net.f_laddr != 0 && f->f_net.f_laddr != ip->ip_dst) {
		return -1;
	}

	return packet_advance(pkt, hlen);
}

char *
inet_ntoa(in_addr_t addr, char *buf, size_t buflen)
{
	addr = ntohl(addr);

	snprintf(buf, buflen, "%u.%u.%u.%u",
	    (uint8_t)(addr >> 24),
	    (uint8_t)(addr >> 16),
	    (uint8_t)(addr >> 8),
	    (uint8_t)(addr));
	return buf;
}
