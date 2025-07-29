/*	From NetBSD: udp.c,v 1.13 2019/03/31 20:08:45 christos Exp	*/

/*
 * Copyright (c) 1992 Regents of the University of California.
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
 *
 * @(#) Header: net.c,v 1.9 93/08/06 19:32:15 leres Exp  (LBL)
 */

#include "sysfile.h"
#include "net.h"

/*
 * UDP protocol header.
 * Per RFC 768, September, 1981.
 */
struct udphdr {
	uint16_t uh_sport;	/* source port */
	uint16_t uh_dport;	/* destination port */
	uint16_t uh_ulen;	/* udp length */
	uint16_t uh_sum;	/* udp checksum */
};

int
udp_send(struct open_file *f, struct packet *pkt)
{
	int error;

	if (!DEV_IS_NETDEV(f->f_dev)) {
		return EOPNOTSUPP;
	}

	struct udphdr uh = {
		.uh_sport = f->f_net.f_lport,
		.uh_dport = f->f_net.f_rport,
	};
	uh.uh_ulen = htons((uint16_t)pkt->pkt_len + sizeof(uh));

	error = packet_prepend(pkt, &uh, sizeof(uh));
	if (error == 0) {
		error = ip_send(f, pkt, IPPROTO_UDP);
	}
	return error;
}

int
udp_recv(struct open_file *f, struct packet *pkt, int tleft)
{
	struct udphdr *uh;
	size_t len;
	int error;

	error = ip_recv(f, pkt, tleft, IPPROTO_UDP);
	if (error) {
		return error;
	}
	error = packet_get_layer(pkt, (void **)&uh, &len);
	if (error) {
		return error;
	}

	if (len < sizeof(*uh)) {
		return -1;
	}
	if (uh->uh_dport != f->f_net.f_lport) {
		return -1;
	}

	len = ntohs(uh->uh_ulen);
	if (len < sizeof(*uh)) {
		return -1;
	}

	return packet_advance(pkt, sizeof(*uh));
}
